using System;
using System.Runtime.InteropServices;
#if NUNITY
using Vector3 = org.critterai.Vector3;
#else
using Vector3 = UnityEngine.Vector3;
#endif

namespace org.critterai.nav.rcn {
    [StructLayout(LayoutKind.Sequential)]
    public struct ConvexVolume {
        const int MAX_CONVEXVOL_PTS = 12;

        [MarshalAs(UnmanagedType.R4, SizeConst = MAX_CONVEXVOL_PTS * 3)]
        public float[] verts;
        public float hmin;
        public float hmax;
        public int nverts;
        public int area;
    }

    internal static class TileCacheEx {
        [DllImport(InteropUtil.PLATFORM_DLL)]
        public static extern NavStatus handleBuild(
            IntPtr contextRoot,
            [In] float[] verts, int nverts,
            ChunkyTriMesh chunkyTriMesh,
            bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
            ConvexVolume convexVolumes, int convexVolumeCount,
	        ref Vector3 bmin, ref Vector3 bmax,
	        float cellSize, float cellHeight,
	        float tileSize,
	        float agentMaxSlope, float agentMaxClimb, float agentRadius, float agentHeight,
	        float edgeMaxLen, float edgeMaxError,
	        float regionMinSize, float regionMergeSize,
	        float vertsPerPoly,
	        float detailSampleDist, float detailSampleMaxError,
	        int maxTiles, int maxPolysPerTile,
	        IntPtr talloc, IntPtr tcomp, IntPtr tmproc,
	        ref IntPtr pTileCache, ref IntPtr pNavMesh, ref IntPtr pNavQuery,
	        IntPtr rasterizeTileLayers);

        [DllImport(InteropUtil.PLATFORM_DLL)]
        public static extern int rasterizeTileLayers(
            IntPtr contextRoot,
            int tx, int ty,
            IntPtr cfg,
            IntPtr tiles,
            int maxTiles,
            [In] float[] verts, int nverts,
            ChunkyTriMesh chunkyTriMesh,
            bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
            ConvexVolume convexVolumes, int convexVolumeCount,
            IntPtr buildTileCacheLayer);
    }
}