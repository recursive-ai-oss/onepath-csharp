// Black-box driver for Recast, Detour and DetourCrowd, written from the public API documentation
// (https://recastnav.com). It builds a navmesh from an OBJ, runs a query set, and steps a crowd, and prints
// bar-format rows (see common.h). Subcommands:
//   build   --obj f.obj [--mode solo|tiled] [--reps N] [settings...]
//   query   --obj f.obj --points f.points [--mode ...] [--reps N] [settings...]
//   crowd   --obj f.obj --points f.points --agents N --steps M [--warmup W] [--dt 0.05] [settings...]
//   tilecache --obj f.obj --points f.points [--obstacles N] [--reps N] [settings...]   (see the block comment below)
// Settings (defaults = the common humanoid values): --cell 0.3 --cellheight 0.2 --agent-height 2.0
//   --agent-radius 0.6 --climb 0.9 --slope 45 --region-min 8 --region-merge 20 --edge-len 12 --edge-err 1.3
//   --verts-per-poly 6 --detail-dist 6 --detail-err 1 --tile-size 64
#include <cmath>
#include <memory>

#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "Recast.h"
#include "common.h"

using namespace harness;

static const char* kSide = "recast-1.6.0";

struct Settings {
  float cs = 0.3f, ch = 0.2f, agentHeight = 2.0f, agentRadius = 0.6f, climb = 0.9f, slope = 45.0f;
  float regionMin = 8, regionMerge = 20, edgeLen = 12, edgeErr = 1.3f, detailDist = 6, detailErr = 1;
  int nvp = 6, tileSize = 64;
  bool tiled = false;
  explicit Settings(const Args& a) {
    cs = (float)a.num("cell", cs); ch = (float)a.num("cellheight", ch);
    agentHeight = (float)a.num("agent-height", agentHeight); agentRadius = (float)a.num("agent-radius", agentRadius);
    climb = (float)a.num("climb", climb); slope = (float)a.num("slope", slope);
    regionMin = (float)a.num("region-min", regionMin); regionMerge = (float)a.num("region-merge", regionMerge);
    edgeLen = (float)a.num("edge-len", edgeLen); edgeErr = (float)a.num("edge-err", edgeErr);
    detailDist = (float)a.num("detail-dist", detailDist); detailErr = (float)a.num("detail-err", detailErr);
    nvp = a.integer("verts-per-poly", nvp); tileSize = a.integer("tile-size", tileSize);
    tiled = a.str("mode", "solo") == "tiled";
  }
  std::string describe() const {
    char b[256];
    std::snprintf(b, sizeof b, "cs=%g ch=%g h=%g r=%g climb=%g slope=%g regmin=%g regmerge=%g edge=%g err=%g nvp=%d dd=%g de=%g%s",
                  cs, ch, agentHeight, agentRadius, climb, slope, regionMin, regionMerge, edgeLen, edgeErr, nvp,
                  detailDist, detailErr, tiled ? " tile=" : "");
    std::string s = b;
    if (tiled) s += std::to_string(tileSize);
    return s;
  }
};

struct TileData { unsigned char* data = nullptr; int size = 0; int tx = 0, ty = 0; };
struct BuildResult {
  std::vector<TileData> tiles;
  long polys = 0, verts = 0;
  long regions = 0, conts = 0, contVerts = 0;  // region and contour stage output (rcCompactHeightfield::maxRegions, rcContourSet)
  double areaXZ = 0;  // sum of polygon areas projected on the ground plane: the walkable coverage
  float bmin[3], bmax[3];
  int tw = 1, th = 1;
};

static void freeResult(BuildResult& r) { for (auto& t : r.tiles) dtFree(t.data); r.tiles.clear(); }

// One tile (or the whole level when solo): Recast's documented build steps, then dtCreateNavMeshData.
static bool buildOne(rcContext& ctx, const Settings& s, const Mesh& m, const float* tbmin, const float* tbmax, int tileX, int tileY,
                     bool tiled, BuildResult& out) {
  rcConfig cfg;
  std::memset(&cfg, 0, sizeof cfg);
  cfg.cs = s.cs; cfg.ch = s.ch; cfg.walkableSlopeAngle = s.slope;
  cfg.walkableHeight = (int)std::ceil(s.agentHeight / s.ch);
  cfg.walkableClimb = (int)std::floor(s.climb / s.ch);
  cfg.walkableRadius = (int)std::ceil(s.agentRadius / s.cs);
  cfg.maxEdgeLen = (int)(s.edgeLen / s.cs);
  cfg.maxSimplificationError = s.edgeErr;
  cfg.minRegionArea = (int)rcSqr(s.regionMin);
  cfg.mergeRegionArea = (int)rcSqr(s.regionMerge);
  cfg.maxVertsPerPoly = s.nvp;
  cfg.detailSampleDist = s.detailDist < 0.9f ? 0 : s.cs * s.detailDist;
  cfg.detailSampleMaxError = s.ch * s.detailErr;
  rcVcopy(cfg.bmin, tbmin); rcVcopy(cfg.bmax, tbmax);
  if (tiled) {
    cfg.tileSize = s.tileSize;
    cfg.borderSize = cfg.walkableRadius + 3;
    cfg.width = cfg.tileSize + cfg.borderSize * 2;
    cfg.height = cfg.tileSize + cfg.borderSize * 2;
    cfg.bmin[0] -= cfg.borderSize * cfg.cs; cfg.bmin[2] -= cfg.borderSize * cfg.cs;
    cfg.bmax[0] += cfg.borderSize * cfg.cs; cfg.bmax[2] += cfg.borderSize * cfg.cs;
  } else {
    rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
  }

  // Triangles overlapping this tile (all of them when solo).
  std::vector<int> tris;
  if (!tiled) tris = m.tris;
  else {
    for (int t = 0; t < m.triCount(); ++t) {
      const int* tr = &m.tris[t * 3];
      float lo0 = 1e30f, lo2 = 1e30f, hi0 = -1e30f, hi2 = -1e30f;
      for (int k = 0; k < 3; ++k) {
        const float* v = &m.verts[tr[k] * 3];
        lo0 = std::min(lo0, v[0]); hi0 = std::max(hi0, v[0]); lo2 = std::min(lo2, v[2]); hi2 = std::max(hi2, v[2]);
      }
      if (hi0 < cfg.bmin[0] || lo0 > cfg.bmax[0] || hi2 < cfg.bmin[2] || lo2 > cfg.bmax[2]) continue;
      tris.insert(tris.end(), tr, tr + 3);
    }
  }
  const int nt = (int)tris.size() / 3;
  if (nt == 0) return true;

  std::vector<unsigned char> areas(nt, 0);
  rcHeightfield* hf = rcAllocHeightfield();
  rcCompactHeightfield* chf = rcAllocCompactHeightfield();
  rcContourSet* cset = rcAllocContourSet();
  rcPolyMesh* pm = rcAllocPolyMesh();
  rcPolyMeshDetail* dm = rcAllocPolyMeshDetail();
  bool ok = false;
  do {
    if (!rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) break;
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, m.verts.data(), m.vertCount(), tris.data(), nt, areas.data());
    if (!rcRasterizeTriangles(&ctx, m.verts.data(), m.vertCount(), tris.data(), areas.data(), nt, *hf, cfg.walkableClimb)) break;
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);
    if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf)) break;
    if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf)) break;
    if (!rcBuildDistanceField(&ctx, *chf)) break;
    if (!rcBuildRegions(&ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) break;
    if (!rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset)) break;
    out.regions += chf->maxRegions;
    out.conts += cset->nconts;
    for (int ci = 0; ci < cset->nconts; ++ci) out.contVerts += cset->conts[ci].nverts;
    if (cset->nconts == 0) { ok = true; break; }
    if (!rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *pm)) break;
    if (!rcBuildPolyMeshDetail(&ctx, *pm, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dm)) break;

    for (int i = 0; i < pm->npolys; ++i) {
      pm->areas[i] = 0;   // one ground area
      pm->flags[i] = 1;   // one walkable flag
    }
    // Coverage: polygon areas projected on the ground plane (voxel units -> world units).
    for (int i = 0; i < pm->npolys; ++i) {
      const unsigned short* p = &pm->polys[i * pm->nvp * 2];
      double a = 0; int n = 0;
      while (n < pm->nvp && p[n] != RC_MESH_NULL_IDX) ++n;
      for (int j = 0, k = n - 1; j < n; k = j++) {
        const unsigned short* vj = &pm->verts[p[j] * 3];
        const unsigned short* vk = &pm->verts[p[k] * 3];
        a += (double)vk[0] * vj[2] - (double)vj[0] * vk[2];
      }
      out.areaXZ += std::fabs(a) * 0.5 * (double)pm->cs * (double)pm->cs;
    }
    out.polys += pm->npolys; out.verts += pm->nverts;

    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof params);
    params.verts = pm->verts; params.vertCount = pm->nverts;
    params.polys = pm->polys; params.polyAreas = pm->areas; params.polyFlags = pm->flags; params.polyCount = pm->npolys;
    params.nvp = pm->nvp;
    params.detailMeshes = dm->meshes; params.detailVerts = dm->verts; params.detailVertsCount = dm->nverts;
    params.detailTris = dm->tris; params.detailTriCount = dm->ntris;
    params.walkableHeight = s.agentHeight; params.walkableRadius = s.agentRadius; params.walkableClimb = s.climb;
    params.tileX = tileX; params.tileY = tileY; params.tileLayer = 0;
    rcVcopy(params.bmin, pm->bmin); rcVcopy(params.bmax, pm->bmax);
    params.cs = cfg.cs; params.ch = cfg.ch; params.buildBvTree = true;
    TileData td; td.tx = tileX; td.ty = tileY;
    if (!dtCreateNavMeshData(&params, &td.data, &td.size)) break;
    out.tiles.push_back(td);
    ok = true;
  } while (false);
  rcFreeHeightField(hf); rcFreeCompactHeightfield(chf); rcFreeContourSet(cset); rcFreePolyMesh(pm); rcFreePolyMeshDetail(dm);
  return ok;
}

static bool buildAll(const Settings& s, const Mesh& m, BuildResult& out) {
  rcContext ctx;
  rcCalcBounds(m.verts.data(), m.vertCount(), out.bmin, out.bmax);
  if (!s.tiled) return buildOne(ctx, s, m, out.bmin, out.bmax, 0, 0, false, out);
  int gw, gh;
  rcCalcGridSize(out.bmin, out.bmax, s.cs, &gw, &gh);
  out.tw = (gw + s.tileSize - 1) / s.tileSize; out.th = (gh + s.tileSize - 1) / s.tileSize;
  const float tcs = s.tileSize * s.cs;
  for (int y = 0; y < out.th; ++y)
    for (int x = 0; x < out.tw; ++x) {
      float lo[3] = {out.bmin[0] + x * tcs, out.bmin[1], out.bmin[2] + y * tcs};
      float hi[3] = {out.bmin[0] + (x + 1) * tcs, out.bmax[1], out.bmin[2] + (y + 1) * tcs};
      if (!buildOne(ctx, s, m, lo, hi, x, y, true, out)) return false;
    }
  return true;
}

static dtNavMesh* makeNavMesh(const Settings& s, BuildResult& r) {
  dtNavMesh* nav = dtAllocNavMesh();
  if (!s.tiled) {
    if (r.tiles.empty()) return nullptr;
    if (dtStatusFailed(nav->init(r.tiles[0].data, r.tiles[0].size, 0))) return nullptr;  // data stays owned by r
    return nav;
  }
  dtNavMeshParams p;
  std::memset(&p, 0, sizeof p);
  rcVcopy(p.orig, r.bmin);
  p.tileWidth = p.tileHeight = s.tileSize * s.cs;
  int tileBits = 1; while ((1 << tileBits) < r.tw * r.th) ++tileBits;
  tileBits = std::min(tileBits, 14);
  p.maxTiles = 1 << tileBits; p.maxPolys = 1 << (22 - tileBits);
  if (dtStatusFailed(nav->init(&p))) return nullptr;
  for (auto& t : r.tiles) nav->addTile(t.data, t.size, 0, 0, nullptr);  // data stays owned by r
  return nav;
}

static int cmdBuild(const Args& a) {
  Mesh m;
  if (!loadObj(a.str("obj"), m)) { std::fprintf(stderr, "cannot read --obj\n"); return 2; }
  Settings s(a);
  const int reps = a.integer("reps", 3);
  const std::string scen = a.str("scenario", a.str("obj") + (s.tiled ? "#tiled" : "#solo"));
  std::vector<double> times;
  BuildResult last; bool have = false;
  for (int i = 0; i < reps; ++i) {
    BuildResult r;
    double t0 = nowMs();
    bool ok = buildAll(s, m, r);
    double t1 = nowMs();
    if (!ok) { std::fprintf(stderr, "build failed\n"); return 1; }
    times.push_back(t1 - t0);
    if (have) freeResult(last);
    last = r; have = true;
  }
  row("build", scen, kSide, "time_best", best(times), "ms");
  row("build", scen, kSide, "time_median", median(times), "ms");
  row("build", scen, kSide, "polys", (double)last.polys, "count");
  row("build", scen, kSide, "verts", (double)last.verts, "count");
  row("build", scen, kSide, "coverage_area", last.areaXZ, "world_units2");
  row("build", scen, kSide, "regions", (double)last.regions, "count");
  row("build", scen, kSide, "contours", (double)last.conts, "count");
  row("build", scen, kSide, "contour_verts", (double)last.contVerts, "count");
  row("build", scen, kSide, "tiles", (double)last.tiles.size(), "count");
  row("build", scen, kSide, "input_tris", (double)m.triCount(), "count");
  row("build", scen, kSide, "peak_rss", peakRssMb(), "MB");
  rowText("build", scen, kSide, "settings", s.describe());
  freeResult(last);
  return 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// tilecache: run-time rebuild with temporary obstacles, through DetourTileCache's documented API.
//   tilecache --obj f.obj --points f.points [--obstacles 16] [--obstacle-radius 2] [--obstacle-height 3] [--reps N]
//             [settings...]   (--tile-size is the tile size in cells, default 64)
// Cache build: every tile is rasterized, filtered, compacted and eroded like a tiled build, cut into height layers
// (rcBuildHeightfieldLayers), stored uncompressed in the cache (a pass-through compressor: the cost of LZ compression is not
// what the rows measure), and turned into navmesh tiles by buildNavMeshTilesAt.
// Rebuild cycle: for each obstacle in turn, add it and update until the cache reports it is up to date; then remove each in the
// same order the same way. Obstacle k stands on the k-th of every (points / obstacles) query points.
// Rows: cache_build_time, time (one cycle), polys_initial, polys (all obstacles in force), polys_restored, coverage_area_initial,
// coverage_area (obstacles in force), coverage_area_restored, tiles, layers.
// ---------------------------------------------------------------------------------------------------------------------
struct PassThroughCompressor : public dtTileCacheCompressor {
  int maxCompressedSize(const int bufferSize) override { return bufferSize; }
  dtStatus compress(const unsigned char* buffer, const int bufferSize, unsigned char* compressed, const int maxCompressedSize,
                    int* compressedSize) override {
    if (bufferSize > maxCompressedSize) return DT_FAILURE;
    std::memcpy(compressed, buffer, (size_t)bufferSize);
    *compressedSize = bufferSize;
    return DT_SUCCESS;
  }
  dtStatus decompress(const unsigned char* compressed, const int compressedSize, unsigned char* buffer, const int maxBufferSize,
                      int* bufferSize) override {
    if (compressedSize > maxBufferSize) return DT_FAILURE;
    std::memcpy(buffer, compressed, (size_t)compressedSize);
    *bufferSize = compressedSize;
    return DT_SUCCESS;
  }
};

// One ground area and one walkable flag, as every other driver sets on its polygons.
struct GroundMeshProcess : public dtTileCacheMeshProcess {
  void process(dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags) override {
    for (int i = 0; i < params->polyCount; ++i) { polyAreas[i] = 0; polyFlags[i] = 1; }
  }
};

struct MeshStats { long polys = 0; double areaXZ = 0; int tiles = 0; };

static MeshStats measureNav(const dtNavMesh* nav) {
  MeshStats st;
  for (int i = 0; i < nav->getMaxTiles(); ++i) {
    const dtMeshTile* t = nav->getTile(i);
    if (!t || !t->header) continue;
    ++st.tiles;
    for (int p = 0; p < t->header->polyCount; ++p) {
      const dtPoly* poly = &t->polys[p];
      if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
      ++st.polys;
      double a = 0;
      for (int j = 0, k = poly->vertCount - 1; j < poly->vertCount; k = j++) {
        const float* vj = &t->verts[poly->verts[j] * 3];
        const float* vk = &t->verts[poly->verts[k] * 3];
        a += (double)vk[0] * vj[2] - (double)vj[0] * vk[2];
      }
      st.areaXZ += std::fabs(a) * 0.5;
    }
  }
  return st;
}

struct CacheSetup {
  dtNavMesh* nav = nullptr;
  dtTileCache* tc = nullptr;
  long layers = 0;
};

// Height layers of one tile into the cache (the tile-mesh steps up to erosion, then rcBuildHeightfieldLayers).
static bool addTileLayers(rcContext& ctx, const Settings& s, const Mesh& m, const float* tbmin, const float* tbmax, int tx, int ty,
                          dtTileCache* tc, dtTileCacheCompressor* comp, long& layerCount) {
  rcConfig cfg;
  std::memset(&cfg, 0, sizeof cfg);
  cfg.cs = s.cs; cfg.ch = s.ch; cfg.walkableSlopeAngle = s.slope;
  cfg.walkableHeight = (int)std::ceil(s.agentHeight / s.ch);
  cfg.walkableClimb = (int)std::floor(s.climb / s.ch);
  cfg.walkableRadius = (int)std::ceil(s.agentRadius / s.cs);
  rcVcopy(cfg.bmin, tbmin); rcVcopy(cfg.bmax, tbmax);
  cfg.tileSize = s.tileSize;
  cfg.borderSize = cfg.walkableRadius + 3;
  cfg.width = cfg.tileSize + cfg.borderSize * 2;
  cfg.height = cfg.tileSize + cfg.borderSize * 2;
  cfg.bmin[0] -= cfg.borderSize * cfg.cs; cfg.bmin[2] -= cfg.borderSize * cfg.cs;
  cfg.bmax[0] += cfg.borderSize * cfg.cs; cfg.bmax[2] += cfg.borderSize * cfg.cs;

  std::vector<int> tris;
  for (int t = 0; t < m.triCount(); ++t) {
    const int* tr = &m.tris[t * 3];
    float lo0 = 1e30f, lo2 = 1e30f, hi0 = -1e30f, hi2 = -1e30f;
    for (int k = 0; k < 3; ++k) {
      const float* v = &m.verts[tr[k] * 3];
      lo0 = std::min(lo0, v[0]); hi0 = std::max(hi0, v[0]); lo2 = std::min(lo2, v[2]); hi2 = std::max(hi2, v[2]);
    }
    if (hi0 < cfg.bmin[0] || lo0 > cfg.bmax[0] || hi2 < cfg.bmin[2] || lo2 > cfg.bmax[2]) continue;
    tris.insert(tris.end(), tr, tr + 3);
  }
  const int nt = (int)tris.size() / 3;
  if (nt == 0) return true;
  std::vector<unsigned char> areas(nt, 0);
  rcHeightfield* hf = rcAllocHeightfield();
  rcCompactHeightfield* chf = rcAllocCompactHeightfield();
  rcHeightfieldLayerSet* lset = rcAllocHeightfieldLayerSet();
  bool ok = false;
  do {
    if (!rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) break;
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, m.verts.data(), m.vertCount(), tris.data(), nt, areas.data());
    if (!rcRasterizeTriangles(&ctx, m.verts.data(), m.vertCount(), tris.data(), areas.data(), nt, *hf, cfg.walkableClimb)) break;
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);
    if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf)) break;
    if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf)) break;
    if (!rcBuildHeightfieldLayers(&ctx, *chf, cfg.borderSize, cfg.walkableHeight, *lset)) break;
    ok = true;
    for (int i = 0; i < lset->nlayers; ++i) {
      const rcHeightfieldLayer* layer = &lset->layers[i];
      dtTileCacheLayerHeader header;
      std::memset(&header, 0, sizeof header);
      header.magic = DT_TILECACHE_MAGIC; header.version = DT_TILECACHE_VERSION;
      header.tx = tx; header.ty = ty; header.tlayer = i;
      rcVcopy(header.bmin, layer->bmin); rcVcopy(header.bmax, layer->bmax);
      header.width = (unsigned char)layer->width; header.height = (unsigned char)layer->height;
      header.minx = (unsigned char)layer->minx; header.maxx = (unsigned char)layer->maxx;
      header.miny = (unsigned char)layer->miny; header.maxy = (unsigned char)layer->maxy;
      header.hmin = (unsigned short)layer->hmin; header.hmax = (unsigned short)layer->hmax;
      unsigned char* data = nullptr; int dataSize = 0;
      if (dtStatusFailed(dtBuildTileCacheLayer(comp, &header, layer->heights, layer->areas, layer->cons, &data, &dataSize))) { ok = false; break; }
      dtCompressedTileRef ref = 0;
      if (dtStatusFailed(tc->addTile(data, dataSize, DT_COMPRESSEDTILE_FREE_DATA, &ref))) { dtFree(data); ok = false; break; }
      ++layerCount;
    }
  } while (false);
  rcFreeHeightField(hf); rcFreeCompactHeightfield(chf); rcFreeHeightfieldLayerSet(lset);
  return ok;
}

static bool buildCache(const Settings& s, const Mesh& m, PassThroughCompressor& comp, GroundMeshProcess& proc, dtTileCacheAlloc& alloc,
                       CacheSetup& out) {
  rcContext ctx;
  float bmin[3], bmax[3];
  rcCalcBounds(m.verts.data(), m.vertCount(), bmin, bmax);
  int gw, gh;
  rcCalcGridSize(bmin, bmax, s.cs, &gw, &gh);
  const int tw = (gw + s.tileSize - 1) / s.tileSize, th = (gh + s.tileSize - 1) / s.tileSize;
  const int maxTiles = tw * th * 4;  // room for several layers per tile
  int tileBits = 1; while ((1 << tileBits) < maxTiles) ++tileBits;
  tileBits = std::min(tileBits, 14);

  dtTileCacheParams tp;
  std::memset(&tp, 0, sizeof tp);
  rcVcopy(tp.orig, bmin);
  tp.cs = s.cs; tp.ch = s.ch; tp.width = s.tileSize; tp.height = s.tileSize;
  tp.walkableHeight = s.agentHeight; tp.walkableRadius = s.agentRadius; tp.walkableClimb = s.climb;
  tp.maxSimplificationError = s.edgeErr;
  tp.maxTiles = 1 << tileBits; tp.maxObstacles = 256;

  dtNavMeshParams np;
  std::memset(&np, 0, sizeof np);
  rcVcopy(np.orig, bmin);
  np.tileWidth = np.tileHeight = s.tileSize * s.cs;
  np.maxTiles = 1 << tileBits; np.maxPolys = 1 << (22 - tileBits);

  out.nav = dtAllocNavMesh();
  if (!out.nav || dtStatusFailed(out.nav->init(&np))) return false;
  out.tc = dtAllocTileCache();
  if (!out.tc || dtStatusFailed(out.tc->init(&tp, &alloc, &comp, &proc))) return false;

  const float tcs = s.tileSize * s.cs;
  for (int y = 0; y < th; ++y)
    for (int x = 0; x < tw; ++x) {
      float lo[3] = {bmin[0] + x * tcs, bmin[1], bmin[2] + y * tcs};
      float hi[3] = {bmin[0] + (x + 1) * tcs, bmax[1], bmin[2] + (y + 1) * tcs};
      if (!addTileLayers(ctx, s, m, lo, hi, x, y, out.tc, &comp, out.layers)) return false;
    }
  for (int y = 0; y < th; ++y)
    for (int x = 0; x < tw; ++x)
      if (dtStatusFailed(out.tc->buildNavMeshTilesAt(x, y, out.nav))) return false;
  return true;
}

static bool settle(dtTileCache* tc, dtNavMesh* nav) {
  bool upToDate = false;
  for (int guard = 0; !upToDate && guard < 100000; ++guard)
    if (dtStatusFailed(tc->update(0.0f, nav, &upToDate))) return false;
  return upToDate;
}

static int cmdTilecache(const Args& a) {
  Mesh m;
  if (!loadObj(a.str("obj"), m)) { std::fprintf(stderr, "cannot read --obj\n"); return 2; }
  std::vector<float> pts = loadPoints(a.str("points"));
  Settings s(a);
  const int reps = a.integer("reps", 3);
  const int count = a.integer("obstacles", 16);
  const float radius = (float)a.num("obstacle-radius", 2.0), height = (float)a.num("obstacle-height", 3.0);
  const std::string scen = a.str("scenario", a.str("obj") + "#tilecache");
  const int npts = (int)(pts.size() / 3);
  if (npts < count) { std::fprintf(stderr, "need %d points\n", count); return 2; }
  // Obstacle k: a cylinder whose base sits half a unit under the k-th stride's point.
  std::vector<float> pos;
  for (int k = 0; k < count; ++k) {
    const float* p = &pts[3 * (k * (npts / count))];
    pos.push_back(p[0]); pos.push_back(p[1] - 0.5f); pos.push_back(p[2]);
  }

  PassThroughCompressor comp; GroundMeshProcess proc; dtTileCacheAlloc alloc;
  std::vector<double> buildTimes, cycleTimes;
  MeshStats initial, stacked, restored;
  long layers = 0;
  CacheSetup cs;
  for (int rep = 0; rep < reps; ++rep) {
    CacheSetup c;
    double t0 = nowMs();
    bool ok = buildCache(s, m, comp, proc, alloc, c);
    buildTimes.push_back(nowMs() - t0);
    if (!ok) { std::fprintf(stderr, "cache build failed\n"); return 1; }
    layers = c.layers;
    if (cs.tc) { dtFreeTileCache(cs.tc); dtFreeNavMesh(cs.nav); }
    cs = c;
  }
  initial = measureNav(cs.nav);
  for (int rep = 0; rep < reps; ++rep) {
    std::vector<dtObstacleRef> refs(count);
    double t0 = nowMs();
    for (int k = 0; k < count; ++k) {
      if (dtStatusFailed(cs.tc->addObstacle(&pos[3 * k], radius, height, &refs[k]))) { std::fprintf(stderr, "addObstacle failed\n"); return 1; }
      if (!settle(cs.tc, cs.nav)) { std::fprintf(stderr, "update failed\n"); return 1; }
    }
    if (rep == 0) stacked = measureNav(cs.nav);
    for (int k = 0; k < count; ++k) {
      if (dtStatusFailed(cs.tc->removeObstacle(refs[k]))) { std::fprintf(stderr, "removeObstacle failed\n"); return 1; }
      if (!settle(cs.tc, cs.nav)) { std::fprintf(stderr, "update failed\n"); return 1; }
    }
    cycleTimes.push_back(nowMs() - t0);
    if (rep == 0) restored = measureNav(cs.nav);
  }
  row("tilecache", scen, kSide, "time_best", best(cycleTimes), "ms");
  row("tilecache", scen, kSide, "time_median", median(cycleTimes), "ms");
  row("tilecache", scen, kSide, "cache_build_time", best(buildTimes), "ms");
  row("tilecache", scen, kSide, "obstacles", (double)count, "count");
  row("tilecache", scen, kSide, "polys_initial", (double)initial.polys, "count");
  row("tilecache", scen, kSide, "polys", (double)stacked.polys, "count");
  row("tilecache", scen, kSide, "polys_restored", (double)restored.polys, "count");
  row("tilecache", scen, kSide, "coverage_area_initial", initial.areaXZ, "world_units2");
  row("tilecache", scen, kSide, "coverage_area", stacked.areaXZ, "world_units2");
  row("tilecache", scen, kSide, "coverage_area_restored", restored.areaXZ, "world_units2");
  row("tilecache", scen, kSide, "tiles", (double)initial.tiles, "count");
  row("tilecache", scen, kSide, "layers", (double)layers, "count");
  row("tilecache", scen, kSide, "input_tris", (double)m.triCount(), "count");
  row("tilecache", scen, kSide, "peak_rss", peakRssMb(), "MB");
  rowText("tilecache", scen, kSide, "settings", s.describe());
  dtFreeTileCache(cs.tc); dtFreeNavMesh(cs.nav);
  return 0;
}

struct QueryPoint { dtPolyRef ref; float pos[3]; };

static std::vector<QueryPoint> snapPoints(dtNavMeshQuery* q, const std::vector<float>& pts) {
  std::vector<QueryPoint> out;
  dtQueryFilter f;
  const float ext[3] = {2, 4, 2};
  for (size_t i = 0; i + 2 < pts.size(); i += 3) {
    QueryPoint qp{};
    dtPolyRef ref = 0; float np[3];
    if (dtStatusSucceed(q->findNearestPoly(&pts[i], ext, &f, &ref, np)) && ref) {
      qp.ref = ref; dtVcopy(qp.pos, np);
      out.push_back(qp);
    } else { qp.ref = 0; out.push_back(qp); }  // keep indices aligned; ref 0 = unusable
  }
  return out;
}

static int cmdQuery(const Args& a) {
  Mesh m;
  if (!loadObj(a.str("obj"), m)) { std::fprintf(stderr, "cannot read --obj\n"); return 2; }
  std::vector<float> pts = loadPoints(a.str("points"));
  if (pts.size() < 6) { std::fprintf(stderr, "need --points with >= 2 points\n"); return 2; }
  Settings s(a);
  const int reps = a.integer("reps", 5);
  const std::string scen = a.str("scenario", a.str("obj") + (s.tiled ? "#tiled" : "#solo"));
  BuildResult r;
  if (!buildAll(s, m, r)) { std::fprintf(stderr, "build failed\n"); return 1; }
  dtNavMesh* nav = makeNavMesh(s, r);
  if (!nav) { std::fprintf(stderr, "navmesh init failed\n"); return 1; }
  dtNavMeshQuery* q = dtAllocNavMeshQuery();
  q->init(nav, 16384);
  dtQueryFilter filter;
  std::vector<QueryPoint> sp = snapPoints(q, pts);
  std::vector<std::pair<int, int>> pairs;
  for (size_t i = 0; i + 1 < sp.size(); i += 2) if (sp[i].ref && sp[i + 1].ref) pairs.push_back({(int)i, (int)i + 1});
  row("query", scen, kSide, "points_snapped", (double)std::count_if(sp.begin(), sp.end(), [](const QueryPoint& p) { return p.ref != 0; }), "count");
  row("query", scen, kSide, "pairs", (double)pairs.size(), "count");

  const int kMax = 4096;
  std::vector<dtPolyRef> path(kMax), spRefs(kMax);
  std::vector<float> straight(kMax * 3);
  std::vector<unsigned char> sflags(kMax);
  double pathLenSum = 0; long pathOk = 0, partial = 0;

  std::vector<double> tPath, tRay, tNear;
  for (int rep = 0; rep < reps; ++rep) {
    pathLenSum = 0; pathOk = 0; partial = 0;
    double t0 = nowMs();
    for (auto& pr : pairs) {
      const QueryPoint& A = sp[pr.first]; const QueryPoint& B = sp[pr.second];
      int pc = 0;
      q->findPath(A.ref, B.ref, A.pos, B.pos, &filter, path.data(), &pc, kMax);
      if (pc <= 0) continue;
      float endPos[3]; dtVcopy(endPos, B.pos);
      if (path[pc - 1] != B.ref) { ++partial; q->closestPointOnPoly(path[pc - 1], B.pos, endPos, nullptr); }
      int sc = 0;
      q->findStraightPath(A.pos, endPos, path.data(), pc, straight.data(), sflags.data(), spRefs.data(), &sc, kMax);
      double len = 0;
      for (int i = 1; i < sc; ++i) len += dtVdist(&straight[(i - 1) * 3], &straight[i * 3]);
      pathLenSum += len; ++pathOk;
    }
    tPath.push_back(nowMs() - t0);
  }
  // HARNESS_DUMP_PATHS=<file>: one line per pair (index, status, corridor polygons, straight-path length, both points) for per-pair comparison.
  if (const char* dumpPath = std::getenv("HARNESS_DUMP_PATHS")) {
    if (FILE* df = std::fopen(dumpPath, "w")) {
      for (auto& pr : pairs) {
        const QueryPoint& A = sp[pr.first]; const QueryPoint& B = sp[pr.second];
        int pc = 0;
        q->findPath(A.ref, B.ref, A.pos, B.pos, &filter, path.data(), &pc, kMax);
        double len = 0; int sc = 0;
        bool part = pc > 0 && path[pc - 1] != B.ref;
        if (pc > 0) {
          float endPos[3]; dtVcopy(endPos, B.pos);
          if (part) q->closestPointOnPoly(path[pc - 1], B.pos, endPos, nullptr);
          q->findStraightPath(A.pos, endPos, path.data(), pc, straight.data(), sflags.data(), spRefs.data(), &sc, kMax);
          for (int i = 1; i < sc; ++i) len += dtVdist(&straight[(i - 1) * 3], &straight[i * 3]);
        }
        std::fprintf(df, "%d %s %d %.9g %g %g %g %g %g %g\n", pr.first / 2, part ? "Partial" : "Success", pc, len,
                     pts[3 * pr.first], pts[3 * pr.first + 1], pts[3 * pr.first + 2], pts[3 * pr.second], pts[3 * pr.second + 1], pts[3 * pr.second + 2]);
      }
      std::fclose(df);
    }
  }
  long hits = 0;
  for (int rep = 0; rep < reps; ++rep) {
    hits = 0;
    double t0 = nowMs();
    for (auto& pr : pairs) {
      const QueryPoint& A = sp[pr.first]; const QueryPoint& B = sp[pr.second];
      float t = 0, hn[3]; int pc = 0;
      q->raycast(A.ref, A.pos, B.pos, &filter, &t, hn, path.data(), &pc, kMax);
      if (t < 1.0f) ++hits;
    }
    tRay.push_back(nowMs() - t0);
  }
  const float ext[3] = {2, 4, 2};
  for (int rep = 0; rep < reps; ++rep) {
    double t0 = nowMs();
    for (size_t i = 0; i + 2 < pts.size(); i += 3) {
      dtPolyRef ref; float np[3];
      q->findNearestPoly(&pts[i], ext, &filter, &ref, np);
    }
    tNear.push_back(nowMs() - t0);
  }
  const double np_ = (double)pairs.size();
  row("query", scen, kSide, "path_time_best", best(tPath), "ms");
  row("query", scen, kSide, "path_per_sec", np_ / (best(tPath) / 1000.0), "1/s");
  row("query", scen, kSide, "path_mean_length", pathOk ? pathLenSum / (double)pathOk : 0, "world_units");
  row("query", scen, kSide, "path_found", (double)pathOk, "count");
  row("query", scen, kSide, "path_partial", (double)partial, "count");
  row("query", scen, kSide, "raycast_time_best", best(tRay), "ms");
  row("query", scen, kSide, "raycast_per_sec", np_ / (best(tRay) / 1000.0), "1/s");
  row("query", scen, kSide, "raycast_hits", (double)hits, "count");
  row("query", scen, kSide, "nearest_time_best", best(tNear), "ms");
  row("query", scen, kSide, "nearest_per_sec", (double)(pts.size() / 3) / (best(tNear) / 1000.0), "1/s");
  row("query", scen, kSide, "peak_rss", peakRssMb(), "MB");
  dtFreeNavMeshQuery(q); dtFreeNavMesh(nav); freeResult(r);
  return 0;
}

static int cmdCrowd(const Args& a) {
  Mesh m;
  if (!loadObj(a.str("obj"), m)) { std::fprintf(stderr, "cannot read --obj\n"); return 2; }
  std::vector<float> pts = loadPoints(a.str("points"));
  Settings s(a);
  const int n = a.integer("agents", 100), steps = a.integer("steps", 100), warm = a.integer("warmup", 20);
  const float dt = (float)a.num("dt", 0.05);
  const std::string scen = a.str("scenario", a.str("obj") + "#crowd" + std::to_string(n));
  BuildResult r;
  if (!buildAll(s, m, r)) { std::fprintf(stderr, "build failed\n"); return 1; }
  dtNavMesh* nav = makeNavMesh(s, r);
  if (!nav) { std::fprintf(stderr, "navmesh init failed\n"); return 1; }
  dtNavMeshQuery* q = dtAllocNavMeshQuery();
  q->init(nav, 2048);
  std::vector<QueryPoint> sp = snapPoints(q, pts);
  std::vector<int> usable;
  for (size_t i = 0; i < sp.size(); ++i) if (sp[i].ref) usable.push_back((int)i);
  if (usable.size() < 2) { std::fprintf(stderr, "not enough usable points\n"); return 1; }

  dtCrowd* crowd = dtAllocCrowd();
  crowd->init(n, s.agentRadius, nav);
  dtCrowdAgentParams ap;
  std::memset(&ap, 0, sizeof ap);
  ap.radius = s.agentRadius; ap.height = s.agentHeight; ap.maxAcceleration = 8.0f; ap.maxSpeed = 3.5f;
  ap.collisionQueryRange = ap.radius * 12.0f; ap.pathOptimizationRange = ap.radius * 30.0f;
  ap.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO | DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_SEPARATION;
  ap.obstacleAvoidanceType = 0; ap.separationWeight = 2.0f;
  int added = 0;
  for (int i = 0; i < n; ++i) {
    const QueryPoint& st = sp[usable[(size_t)i % usable.size()]];
    const QueryPoint& goal = sp[usable[((size_t)i * 7 + usable.size() / 2) % usable.size()]];
    int idx = crowd->addAgent(st.pos, &ap);
    if (idx < 0) continue;
    ++added;
    crowd->requestMoveTarget(idx, goal.ref, goal.pos);
  }
  for (int i = 0; i < warm; ++i) crowd->update(dt, nullptr);
  double t0 = nowMs();
  for (int i = 0; i < steps; ++i) crowd->update(dt, nullptr);
  double ms = nowMs() - t0;
  int moving = 0;
  for (int i = 0; i < crowd->getAgentCount(); ++i) {
    const dtCrowdAgent* ag = crowd->getAgent(i);
    if (ag->active && dtVlen(ag->vel) > 0.1f) ++moving;
  }
  row("crowd", scen, kSide, "agents", (double)added, "count");
  row("crowd", scen, kSide, "steps", (double)steps, "count");
  row("crowd", scen, kSide, "time", ms, "ms");
  row("crowd", scen, kSide, "agents_per_ms", (double)added * steps / ms, "agent-steps/ms");
  row("crowd", scen, kSide, "moving_at_end", (double)moving, "count");
  row("crowd", scen, kSide, "peak_rss", peakRssMb(), "MB");
  dtFreeCrowd(crowd); dtFreeNavMeshQuery(q); dtFreeNavMesh(nav); freeResult(r);
  return 0;
}

int main(int argc, char** argv) {
  Args a(argc, argv);
  if (a.pos.empty()) { std::fprintf(stderr, "usage: recast-harness build|query|crowd|tilecache --obj f.obj ...\n"); return 2; }
  if (a.pos[0] == "build") return cmdBuild(a);
  if (a.pos[0] == "query") return cmdQuery(a);
  if (a.pos[0] == "crowd") return cmdCrowd(a);
  if (a.pos[0] == "tilecache") return cmdTilecache(a);
  std::fprintf(stderr, "unknown command %s\n", a.pos[0].c_str());
  return 2;
}
