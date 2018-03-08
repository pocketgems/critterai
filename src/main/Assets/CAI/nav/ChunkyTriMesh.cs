using System;
using System.Runtime.InteropServices;

namespace org.critterai.nav.rcn {
    [StructLayout(LayoutKind.Sequential)]
	internal struct ChunkyTriMesh
	{
        public IntPtr mNodes;
        public int mNodeCount;
        public IntPtr mTris;
        public int mTriCount;
        public int mMaxTrisPerChunk;
    }
}
