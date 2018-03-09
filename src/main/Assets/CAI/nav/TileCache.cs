using System;

using org.critterai.geom;
using org.critterai.nav.rcn;

#if NUNITY
using Vector3 = org.critterai.Vector3;
#else
using Vector3 = UnityEngine.Vector3;
#endif

namespace org.critterai.nav {
    /// <summary>
    /// ???
    /// </summary>
    public sealed class TileCache {
        internal IntPtr root = IntPtr.Zero;

        internal TileCache(IntPtr root) {
            this.root = root;
        }

        /// <summary>
        /// wtf
        /// </summary>
        /// <param name="contextRoot"></param>
        /// <param name="triangleMesh"></param>
        /// <param name="vertsPerPoly"></param>
        /// <param name="trisPerChunk"></param>
        /// <param name="filterLowHangingObstacles"></param>
        /// <param name="filterLedgeSpans"></param>
        /// <param name="filterWalkableLowHeightSpans"></param>
        /// <param name="cellSize"></param>
        /// <param name="cellHeight"></param>
        /// <param name="tileSize"></param>
        /// <param name="agentMaxSlope"></param>
        /// <param name="agentMaxClimb"></param>
        /// <param name="agentRadius"></param>
        /// <param name="agentHeight"></param>
        /// <param name="edgeMaxLen"></param>
        /// <param name="edgeMaxError"></param>
        /// <param name="regionMinSize"></param>
        /// <param name="regionMergeSize"></param>
        /// <param name="detailSampleDist"></param>
        /// <param name="detailSampleMaxError"></param>
        /// <param name="tileCache"></param>
        /// <param name="navmesh"></param>
        /// <param name="navmeshQuery"></param>
        /// <returns></returns>
        public static NavStatus Create(
            IntPtr contextRoot,
            TriangleMesh triangleMesh, int vertsPerPoly, int trisPerChunk,
            bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
            float cellSize, float cellHeight, float tileSize,
            float agentMaxSlope, float agentMaxClimb, float agentRadius, float agentHeight,
            float edgeMaxLen, float edgeMaxError,
            float regionMinSize, float regionMergeSize,
            float detailSampleDist, float detailSampleMaxError,
            out TileCache tileCache, out Navmesh navmesh, out NavmeshQuery navmeshQuery) {
            var bmin = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            var bmax = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

            foreach (var vert in triangleMesh.verts) {
                bmin.x = Math.Max(vert.x, bmin.x);
                bmin.y = Math.Max(vert.y, bmin.y);
                bmin.z = Math.Max(vert.z, bmin.z);
                bmax.x = Math.Min(vert.x, bmax.x);
                bmax.y = Math.Min(vert.y, bmax.y);
                bmax.z = Math.Min(vert.z, bmax.z);
            }

            var pTileCache = new IntPtr();
            var pNavMesh = new IntPtr();
            var pNavQuery = new IntPtr();

            var status = TileCacheEx.handleBuild(
                buildContext: contextRoot,
                verts: triangleMesh.verts, nverts: triangleMesh.vertCount, vertsPerPoly: vertsPerPoly,
                tris: triangleMesh.tris, ntris: triangleMesh.triCount, trisPerChunk: trisPerChunk,
                filterLowHangingObstacles: filterLowHangingObstacles,
                filterLedgeSpans: filterLedgeSpans,
                filterWalkableLowHeightSpans: filterWalkableLowHeightSpans,
                bmin: ref bmin, bmax: ref bmax,
                cellSize: cellSize, cellHeight: cellHeight,
                tileSize: tileSize,
                agentMaxSlope: agentMaxSlope, agentMaxClimb: agentMaxClimb, agentRadius: agentRadius, agentHeight: agentHeight,
                edgeMaxLen: edgeMaxLen, edgeMaxError: edgeMaxError,
                regionMinSize: regionMinSize, regionMergeSize: regionMergeSize,
                detailSampleDist: detailSampleDist, detailSampleMaxError: detailSampleMaxError,
                pTileCache: ref pTileCache, pNavMesh: ref pNavMesh, pNavQuery: ref pNavQuery,
                rasterizeTileLayers: TileCacheEx.getRasterizeTileLayers());

            if ((status & NavStatus.Sucess) != 0) {
                tileCache = new TileCache(pTileCache);
                navmesh = new Navmesh(pNavMesh);
                navmeshQuery = new NavmeshQuery(pNavQuery, true, interop.AllocType.External);
            } else {
                tileCache = default(TileCache);
                navmesh = default(Navmesh);
                navmeshQuery = default(NavmeshQuery);
            }

            return status;
        }
    }
}