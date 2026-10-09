"""Compute the project -> SME distance matrix and write it into data.js.

  python build_matrix.py --approx   straight-line x detour factor (no downloads, placeholder)
  python build_matrix.py            real road distances via OSMnx (needs internet;
                                    pip install osmnx networkx) -- not yet tested

Run it again whenever SMEs or projects in data.js change.
"""
import json, math, pathlib, sys

P = pathlib.Path(__file__).with_name("data.js")
PREFIX = "window.DATA = "
d = json.loads(P.read_text(encoding="utf-8").split(PREFIX, 1)[1].strip().rstrip(";"))


def hav(a, b):
    """Great-circle distance in km between (lat, lon) pairs."""
    la1, lo1, la2, lo2 = map(math.radians, (*a, *b))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 6371.0 * 2 * math.asin(math.sqrt(h))


def approx(p, s):
    return hav((p["lat"], p["lon"]), (s["lat"], s["lon"])) * d["detour"]


if "--approx" in sys.argv:
    dist = {p["id"]: {s["id"]: round(approx(p, s), 1) for s in d["smes"]} for p in d["projects"]}
    d["distSource"] = "approx"
else:
    import osmnx as ox
    import networkx as nx

    pts = d["projects"] + d["smes"]
    w, e = min(x["lon"] for x in pts) - 0.3, max(x["lon"] for x in pts) + 0.3
    s_, n = min(x["lat"] for x in pts) - 0.3, max(x["lat"] for x in pts) + 0.3
    major = '["highway"~"motorway|trunk|primary|secondary|tertiary"]'  # keeps the graph small
    try:
        G = ox.graph_from_bbox(bbox=(w, s_, e, n), custom_filter=major, retain_all=True)  # osmnx >= 2
    except TypeError:
        G = ox.graph_from_bbox(n, s_, e, w, custom_filter=major, retain_all=True)  # osmnx 1.x
    Gu = G.to_undirected()  # ignores one-way rules; fine for planning distances
    node = lambda x: ox.distance.nearest_nodes(Gu, X=x["lon"], Y=x["lat"])
    sme_nodes = {s["id"]: node(s) for s in d["smes"]}
    dist = {}
    for p in d["projects"]:
        lengths = nx.single_source_dijkstra_path_length(Gu, node(p), weight="length")
        dist[p["id"]] = {
            sid: round(lengths[nd] / 1000, 1) if nd in lengths else round(approx(p, next(s for s in d["smes"] if s["id"] == sid)), 1)
            for sid, nd in sme_nodes.items()
        }
    d["distSource"] = "osmnx"

d["dist"] = dist
P.write_text(PREFIX + json.dumps(d, ensure_ascii=False, indent=1) + ";\n", encoding="utf-8")
print("distSource =", d["distSource"], "| projects:", len(d["projects"]), "| SMEs:", len(d["smes"]))
