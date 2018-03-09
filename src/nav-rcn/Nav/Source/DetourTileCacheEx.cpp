#include "DetourCommon.h"
#include "DetourStatus.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "ChunkyTriMesh.h"
#include <string.h>

struct Compressor : public dtTileCacheCompressor
{
	static Compressor instance;

	virtual int maxCompressedSize(const int bufferSize)
	{
		return bufferSize;
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = bufferSize;
		memcpy(compressed, buffer, bufferSize);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = compressedSize;
		memcpy(buffer, compressed, compressedSize);
		return DT_SUCCESS;
	}
};

Compressor Compressor::instance;

struct LinearAllocator : public dtTileCacheAlloc
{
	static LinearAllocator instance;

	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

LinearAllocator LinearAllocator::instance(32000);

struct MeshProcess : public dtTileCacheMeshProcess
{
	static MeshProcess instance;

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// do nothing
	}
};

MeshProcess MeshProcess::instance;


static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize * 4;
}

bool buildTileCacheLayer(
	int tx, int ty, int i,
	const float *bmin, const float *bmax,
	int width, int height, int minx, int maxx, int miny, int maxy, int hmin, int hmax,
	unsigned char *heights, unsigned char *areas, unsigned char *cons,
	TileCacheData *tile) {
	// Store header
	dtTileCacheLayerHeader header;
	header.magic = DT_TILECACHE_MAGIC;
	header.version = DT_TILECACHE_VERSION;

	// Tile layer location in the navmesh.
	header.tx = tx;
	header.ty = ty;
	header.tlayer = i;
	dtVcopy(header.bmin, /*layer->*/bmin);
	dtVcopy(header.bmax, /*layer->*/bmax);

	// Tile info.
	header.width = (unsigned char)/*layer->*/width;
	header.height = (unsigned char)/*layer->*/height;
	header.minx = (unsigned char)/*layer->*/minx;
	header.maxx = (unsigned char)/*layer->*/maxx;
	header.miny = (unsigned char)/*layer->*/miny;
	header.maxy = (unsigned char)/*layer->*/maxy;
	header.hmin = (unsigned short)/*layer->*/hmin;
	header.hmax = (unsigned short)/*layer->*/hmax;

	dtStatus status = dtBuildTileCacheLayer(
		&Compressor::instance,
		&header,
		/*layer->*/heights, /*layer->*/areas, /*layer->*/cons,
		&tile->data, &tile->dataSize);
	if (dtStatusFailed(status))
	{
		return false;
	}
	return true;
}

dtStatus handleBuild(
	void *pCtx,
	float *verts, int nverts, int vertsPerPoly,
	int *tris, int ntris, int trisPerChunk,
	bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
	const float *bmin, const float *bmax,
	float cellSize, float cellHeight,
	float tileSize,
	float agentMaxSlope, float agentMaxClimb, float agentRadius, float agentHeight,
	float edgeMaxLen, float edgeMaxError,
	float regionMinSize, float regionMergeSize,
	float detailSampleDist, float detailSampleMaxError,
	dtTileCache **pTileCache, dtNavMesh **pNavMesh, dtNavMeshQuery **pNavQuery,
	dtStatus (*rasterizeTileLayers)(
		void *pCtx,
		int tx, int ty,
		rcConfig *cfg,
		TileCacheData* tiles, int maxTiles,
		float *verts, int nverts,
		rcChunkyTriMesh *chunkyMesh,
		bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
		bool (*buildTileCacheLayer)(
			int tx, int ty, int i,
			const float *bmin, const float *bmax,
			int width, int height, int minx, int maxx, int miny, int maxy, int hmin, int hmax,
			unsigned char *heights, unsigned char *areas, unsigned char *cons,
			TileCacheData *tile)))
{
	/*
	if (!m_geom || !m_geom->getMesh())
	{
	//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
	return false;
	}

	tmproc->init(m_geom);
	*/
	// Init cache
	//const float* bmin = m_geom->getNavMeshBoundsMin();
	//const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	dtCalcGridSize(bmin, bmax, cellSize, &gw, &gh);
	const int ts = (int)tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	int tileBits = dtMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	int maxTiles = 1 << tileBits;
	int maxPolysPerTile = 1 << polyBits;

	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	dtVcopy(tcparams.orig, bmin);
	tcparams.cs = cellSize;
	tcparams.ch = cellHeight;
	tcparams.width = (int)tileSize;
	tcparams.height = (int)tileSize;
	tcparams.walkableHeight = agentHeight;
	tcparams.walkableRadius = agentRadius;
	tcparams.walkableClimb = agentMaxClimb;
	tcparams.maxSimplificationError = edgeMaxError;
	tcparams.maxTiles = tw * th*EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = 128;

	dtFreeTileCache(*pTileCache);

	*pTileCache = dtAllocTileCache();
	dtTileCache *tileCache = *pTileCache;
	if (!tileCache)
	{
		//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	dtStatus status = tileCache->init(&tcparams, &LinearAllocator::instance, &Compressor::instance, &MeshProcess::instance);
	if (dtStatusFailed(status))
	{
		//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return status;
	}

	dtFreeNavMesh(*pNavMesh);

	*pNavMesh = dtAllocNavMesh();
	dtNavMesh *navMesh = *pNavMesh;
	if (!navMesh)
	{
		//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	dtVcopy(params.orig, bmin);
	params.tileWidth = tileSize * cellSize;
	params.tileHeight = tileSize * cellSize;
	params.maxTiles = maxTiles;
	params.maxPolys = maxPolysPerTile;

	status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return status;
	}

	*pNavQuery = dtAllocNavMeshQuery();
	dtNavMeshQuery *navQuery = *pNavQuery;
	if (!navQuery) {
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	status = navQuery->init(navMesh, 2048);
	if (dtStatusFailed(status))
	{
		//m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return status;
	}


	// Preprocess tiles.

	//m_ctx->resetTimers();

	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = cellSize;
	cfg.ch = cellHeight;
	cfg.walkableSlopeAngle = agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(edgeMaxLen / cellSize);
	cfg.maxSimplificationError = edgeMaxError;
	cfg.minRegionArea = (int)dtSqr(regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)dtSqr(regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = vertsPerPoly;
	cfg.tileSize = (int)tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	cfg.detailSampleMaxError = cellHeight * detailSampleMaxError;
	dtVcopy(cfg.bmin, bmin);
	dtVcopy(cfg.bmax, bmax);

	int cacheLayerCount = 0;
	int cacheCompressedSize = 0;
	int cacheRawSize = 0;

	rcChunkyTriMesh chunkyTriMesh;
	if (!rcCreateChunkyTriMesh(verts, tris, ntris, trisPerChunk, &chunkyTriMesh)) {
		return DT_FAILURE;
	}

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(pCtx, x, y, &cfg, tiles, MAX_LAYERS, verts, nverts, &chunkyTriMesh, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, buildTileCacheLayer);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}

				cacheLayerCount++;
				cacheCompressedSize += tile->dataSize;
				cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}

	// Build initial meshes
	//m_ctx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			tileCache->buildNavMeshTilesAt(x, y, navMesh);
	//m_ctx->stopTimer(RC_TIMER_TOTAL);

	//m_cacheBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	//m_cacheBuildMemUsage = talloc->high;


	const dtNavMesh* nav = navMesh;
	int navmeshMemUsage = 0;
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nav->getTile(i);
		if (tile->header)
			navmeshMemUsage += tile->dataSize;
	}
	//printf("navmeshMemUsage = %.1f kB", navmeshMemUsage / 1024.0f);

	/*
	if (m_tool)
	m_tool->init(this);
	initToolStates(this);
	*/
	return DT_SUCCESS;
}