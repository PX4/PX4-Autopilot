#include <cuda_runtime_api.h>
#include <vector>
#include <string>
#include <fstream>
#include <cstdio>
/*
ls *.dot | xargs -I{} dot -Tsvg {} -o {}.svg
sudo apt install graphviz
*/

#ifndef CGCHK
#define CGCHK(call) do { \
  cudaError_t _e = (call); \
  if (_e != cudaSuccess) { \
  } \
} while(0)
#endif

static const char* nodeTypeName(cudaGraphNodeType t){
  switch (t){
    case cudaGraphNodeTypeKernel:    return "Kernel";
    case cudaGraphNodeTypeMemcpy:    return "Memcpy";
    case cudaGraphNodeTypeMemset:    return "Memset";
    case cudaGraphNodeTypeHost:      return "Host";
    case cudaGraphNodeTypeGraph:     return "ChildGraph";
    case cudaGraphNodeTypeEmpty:     return "Empty";
    case cudaGraphNodeTypeWaitEvent: return "WaitEvent";
    case cudaGraphNodeTypeEventRecord:return "EventRecord";
    case cudaGraphNodeTypeMemAlloc:  return "MemAlloc";
    case cudaGraphNodeTypeMemFree:   return "MemFree";
    default:                         return "Unknown";
  }
}

static void dumpCudaGraphDOT(cudaGraph_t graph, const char* filepath){
  size_t nNodes = 0, nEdges = 0;
  CGCHK(cudaGraphGetNodes(graph, nullptr, &nNodes));
  std::vector<cudaGraphNode_t> nodes(nNodes);
  if (nNodes) CGCHK(cudaGraphGetNodes(graph, nodes.data(), &nNodes));

  CGCHK(cudaGraphGetEdges(graph, nullptr, nullptr, &nEdges));
  std::vector<cudaGraphNode_t> from(nEdges), to(nEdges);
  if (nEdges) CGCHK(cudaGraphGetEdges(graph, from.data(), to.data(), &nEdges));

  auto idxOf = [&](cudaGraphNode_t h)->int{
    for (size_t i=0;i<nodes.size();++i) if (nodes[i]==h) return (int)i;
    return -1;
  };

  std::ofstream os(filepath);
  os << "digraph cuda_graph {\n  rankdir=LR;\n  node [shape=box];\n";

  for (size_t i=0;i<nNodes;++i){
    cudaGraphNodeType t; CGCHK(cudaGraphNodeGetType(nodes[i], &t));
    std::string label = nodeTypeName(t);

    char buf[256]; buf[0]='\0';
    if (t == cudaGraphNodeTypeKernel){
      cudaKernelNodeParams p{};
      CGCHK(cudaGraphKernelNodeGetParams(nodes[i], &p));
      std::snprintf(buf, sizeof(buf),
        "\\ngrid=(%u,%u,%u) block=(%u,%u,%u)",
        p.gridDim.x, p.gridDim.y, p.gridDim.z,
        p.blockDim.x, p.blockDim.y, p.blockDim.z);
      label += buf;
    } else if (t == cudaGraphNodeTypeMemcpy){
      cudaMemcpy3DParms mp{}; CGCHK(cudaGraphMemcpyNodeGetParams(nodes[i], &mp));
      size_t w = mp.extent.width  ? mp.extent.width  : 1;
      size_t h = mp.extent.height ? mp.extent.height : 1;
      size_t d = mp.extent.depth  ? mp.extent.depth  : 1;
      std::snprintf(buf, sizeof(buf), "\\nbytes≈%zu", w*h*d);
      label += buf;
    } else if (t == cudaGraphNodeTypeMemset){
      cudaMemsetParams sp{}; CGCHK(cudaGraphMemsetNodeGetParams(nodes[i], &sp));
      size_t bytes = size_t(sp.elementSize) * sp.width * sp.height;
      std::snprintf(buf, sizeof(buf), "\\nbytes=%zu val=%d", bytes, int(sp.value));
      label += buf;
    }

    os << "  n" << i << " [label=\"" << label << "\"];\n";
  }

  for (size_t e=0;e<nEdges;++e){
    int u = idxOf(from[e]), v = idxOf(to[e]);
    if (u>=0 && v>=0) os << "  n" << u << " -> n" << v << ";\n";
  }
  os << "}\n";
  // std::cerr << "Wrote DOT to: " << filepath << " ("<< nNodes <<" nodes, " << nEdges << " edges)\n";
}

