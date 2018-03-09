using System;
using System.Runtime.InteropServices;
#if NUNITY
using Vector3 = org.critterai.Vector3;
#else
using Vector3 = UnityEngine.Vector3;
#endif

namespace org.critterai.nav.rcn {
    internal static class TileCacheEx {
        [DllImport(InteropUtil.PLATFORM_DLL)]
        public static extern NavStatus handleBuild(
            IntPtr buildContext,
            [In] Vector3[] verts, int nverts, int vertsPerPoly,
            [In] int[] tris, int ntris, int trisPerChunk,
            bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
	        ref Vector3 bmin, ref Vector3 bmax,
	        float cellSize, float cellHeight,
	        float tileSize,
            float agentMaxSlope, float agentMaxClimb, float agentRadius, float agentHeight,
	        float edgeMaxLen, float edgeMaxError,
	        float regionMinSize, float regionMergeSize,
	        float detailSampleDist, float detailSampleMaxError,
	        ref IntPtr pTileCache, ref IntPtr pNavMesh, ref IntPtr pNavQuery,
            IntPtr rasterizeTileLayers);

        [DllImport(InteropUtil.PLATFORM_DLL)]
        public static extern IntPtr getRasterizeTileLayers();
    }
}