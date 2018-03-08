#include "DetourCommon.h"
#include "DetourStatus.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "ChunkyTriMesh.h"
#include <string.h>

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
	const unsigned char *heights, const unsigned char *areas, const unsigned char *cons,
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

	dtTileCacheCompressor *comp; //TODO
	dtStatus status = dtBuildTileCacheLayer(comp, &header, /*layer->*/heights, /*layer->*/areas, /*layer->*/cons,
		&tile->data, &tile->dataSize);
	if (dtStatusFailed(status))
	{
		return false;
	}
	return true;
}

dtStatus handleBuild(
	void *pCtx,
	const float *verts, const int nverts,
	const rcChunkyTriMesh* chunkyMesh,
	bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
	const ConvexVolume* convexVolumes, int convexVolumeCount,
	const float* bmin, const float* bmax,
	float cellSize, float cellHeight,
	float tileSize,
	float agentMaxSlope, float agentMaxClimb, float agentRadius, float agentHeight,
	float edgeMaxLen, float edgeMaxError,
	float regionMinSize, float regionMergeSize,
	float vertsPerPoly,
	float detailSampleDist, float detailSampleMaxError,
	int maxTiles, int maxPolysPerTile,
	dtTileCacheAlloc *talloc, dtTileCacheCompressor *tcomp, dtTileCacheMeshProcess *tmproc,
	dtTileCache **pTileCache, dtNavMesh **pNavMesh, dtNavMeshQuery **pNavQuery,
	dtStatus (*rasterizeTileLayers)(
		void *pCtx,
		const int tx, const int ty,
		const rcConfig& cfg,
		TileCacheData* tiles,
		const int maxTiles,
		const float *verts, const int nverts, const void* pChunkyMesh,
		bool m_filterLowHangingObstacles, bool m_filterLedgeSpans, bool m_filterWalkableLowHeightSpans,
		const void* pConvexVolumes, int convexVolumeCount,
		bool (*buildTileCacheLayer)(
			int tx, int ty, int i,
			const float *bmin, const float *bmax,
			int width, int height, int minx, int maxx, int miny, int maxy, int hmin, int hmax,
			const unsigned char *heights, const unsigned char *areas, const unsigned char *cons,
			TileCacheData *tile)))
{
	dtStatus status;
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
	cfg.maxVertsPerPoly = (int)vertsPerPoly;
	cfg.tileSize = (int)tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	cfg.detailSampleMaxError = cellHeight * detailSampleMaxError;
	dtVcopy(cfg.bmin, bmin);
	dtVcopy(cfg.bmax, bmax);

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
	status = tileCache->init(&tcparams, talloc, tcomp, tmproc);
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

	int cacheLayerCount = 0;
	int cacheCompressedSize = 0;
	int cacheRawSize = 0;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(pCtx, x, y, cfg, tiles, MAX_LAYERS, verts, nverts, chunkyMesh, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, convexVolumes, convexVolumeCount, buildTileCacheLayer);

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