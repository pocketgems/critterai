#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"



typedef unsigned int dtObstacleRef;

typedef unsigned int dtCompressedTileRef;

/// Flags for addTile
enum dtCompressedTileFlags
{
	DT_COMPRESSEDTILE_FREE_DATA = 0x01,					///< Navmesh owns the tile memory and should free it.
};

struct dtCompressedTile
{
	unsigned int salt;						///< Counter describing modifications to the tile.
	struct dtTileCacheLayerHeader* header;
	unsigned char* compressed;
	int compressedSize;
	unsigned char* data;
	int dataSize;
	unsigned int flags;
	dtCompressedTile* next;
};

enum ObstacleState
{
	DT_OBSTACLE_EMPTY,
	DT_OBSTACLE_PROCESSING,
	DT_OBSTACLE_PROCESSED,
	DT_OBSTACLE_REMOVING,
};

enum ObstacleType
{
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX, // AABB
	DT_OBSTACLE_ORIENTED_BOX, // OBB
};

struct dtObstacleCylinder
{
	float pos[ 3 ];
	float radius;
	float height;
};

struct dtObstacleBox
{
	float bmin[ 3 ];
	float bmax[ 3 ];
};

struct dtObstacleOrientedBox
{
	float center[ 3 ];
	float halfExtents[ 3 ];
	float rotAux[ 2 ]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;

static const int DT_MAX_TOUCHED_TILES = 8;
struct dtTileCacheObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
	};

	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	unsigned short salt;
	unsigned char type;
	unsigned char state;
	unsigned char ntouched;
	unsigned char npending;
	dtTileCacheObstacle* next;
};

struct dtTileCacheParams
{
	float orig[3];
	float cs, ch;
	int width, height;
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float maxSimplificationError;
	int maxTiles;
	int maxObstacles;
};

/// Specifies a configuration to use when performing Recast builds.
/// @ingroup recast
struct rcConfig
{
	/// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	int width;

	/// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	int height;

	/// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	int tileSize;

	/// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	int borderSize;

	/// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] 
	float cs;

	/// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	float ch;

	/// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	float bmin[3];

	/// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	float bmax[3];

	/// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] 
	float walkableSlopeAngle;

	/// Minimum floor to 'ceiling' height that will still allow the floor area to 
	/// be considered walkable. [Limit: >= 3] [Units: vx] 
	int walkableHeight;

	/// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] 
	int walkableClimb;

	/// The distance to erode/shrink the walkable area of the heightfield away from 
	/// obstructions.  [Limit: >=0] [Units: vx] 
	int walkableRadius;

	/// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] 
	int maxEdgeLen;

	/// The maximum distance a simplfied contour's border edges should deviate 
	/// the original raw contour. [Limit: >=0] [Units: vx]
	float maxSimplificationError;

	/// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] 
	int minRegionArea;

	/// Any regions with a span count smaller than this value will, if possible, 
	/// be merged with larger regions. [Limit: >=0] [Units: vx] 
	int mergeRegionArea;

	/// The maximum number of vertices allowed for polygons generated during the 
	/// contour to polygon conversion process. [Limit: >= 3] 
	int maxVertsPerPoly;

	/// Sets the sampling distance to use when generating the detail mesh.
	/// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu] 
	float detailSampleDist;

	/// The maximum distance the detail mesh surface should deviate from heightfield
	/// data. (For height detail only.) [Limit: >=0] [Units: wu] 
	float detailSampleMaxError;
};

struct dtTileCacheMeshProcess
{
	virtual ~dtTileCacheMeshProcess() { }

	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags) = 0;
};


class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();
	
	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	const dtTileCacheParams* getParams() const { return &m_params; }
	
	inline int getTileCount() const { return m_params.maxTiles; }
	inline const dtCompressedTile* getTile(const int i) const { return &m_tiles[i]; }
	
	inline int getObstacleCount() const { return m_params.maxObstacles; }
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }
	
	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);
	
	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;
	
	dtStatus init(const dtTileCacheParams* params,
				  struct dtTileCacheAlloc* talloc,
				  struct dtTileCacheCompressor* tcomp,
				  struct dtTileCacheMeshProcess* tmproc);
	
	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const ;
	
	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;
	
	dtStatus addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result);
	
	dtStatus removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize);
	
	// Cylinder obstacle.
	dtStatus addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);

	// Aabb obstacle.
	dtStatus addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	dtStatus addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result);
	
	dtStatus removeObstacle(const dtObstacleRef ref);
	
	dtStatus queryTiles(const float* bmin, const float* bmax,
						dtCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	/// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	///  @param[in]		dt			The time step size. Currently not used.
	///  @param[in]		navmesh		The mesh to affect when rebuilding tiles.
	///  @param[out]	upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
	///  							If the tile cache is up to date another (immediate) call to update will have no effect;
	///  							otherwise another call will continue processing obstacle requests and tile rebuilds.
	dtStatus update(const float dt, class dtNavMesh* navmesh, bool* upToDate = 0);
	
	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	
	dtStatus buildNavMeshTile(const dtCompressedTileRef ref, class dtNavMesh* navmesh);
	
	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, float* bmin, float* bmax) const;
	
	void getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const;
	

	/// Encodes a tile id.
	inline dtCompressedTileRef encodeTileId(unsigned int salt, unsigned int it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}
	
	/// Decodes a tile salt.
	inline unsigned int decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> m_tileBits) & saltMask);
	}
	
	/// Decodes a tile id.
	inline unsigned int decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1<<m_tileBits)-1;
		return (unsigned int)(ref & tileMask);
	}

	/// Encodes an obstacle id.
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}
	
	/// Decodes an obstacle salt.
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)((ref >> 16) & saltMask);
	}
	
	/// Decodes an obstacle id.
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)(ref & tileMask);
	}
	
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&);
	dtTileCache& operator=(const dtTileCache&);

	enum ObstacleRequestAction
	{
		REQUEST_ADD,
		REQUEST_REMOVE,
	};
	
	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};
	
	int m_tileLutSize;						///< Tile hash lookup size (must be pot).
	int m_tileLutMask;						///< Tile hash lookup mask.
	
	dtCompressedTile** m_posLookup;			///< Tile hash lookup.
	dtCompressedTile* m_nextFreeTile;		///< Freelist of tiles.
	dtCompressedTile* m_tiles;				///< List of tiles.
	
	unsigned int m_saltBits;				///< Number of salt bits in the tile ID.
	unsigned int m_tileBits;				///< Number of tile bits in the tile ID.
	
	dtTileCacheParams m_params;
	
	dtTileCacheAlloc* m_talloc;
	dtTileCacheCompressor* m_tcomp;
	dtTileCacheMeshProcess* m_tmproc;
	
	dtTileCacheObstacle* m_obstacles;
	dtTileCacheObstacle* m_nextFreeObstacle;
	
	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_reqs[MAX_REQUESTS];
	int m_nreqs;
	
	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif
