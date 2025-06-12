// the d3.js script might not yet be loaded (because it's not in <head>), so we
// wrap everything in a function and retry until d3 is available
function initializeGraph() {
    if (typeof d3 === 'undefined') {
        // try again later
        setTimeout(initializeGraph, 500);
        return;
    }


var graph_option = document.getElementById("select-graph");
var default_json_file = graph_option.value;
var minOpacity = 0.1; // opacity when a node/link is faded

/* search field: highlight all matching nodes on text change */
var g_filterText = "";
function searchTextChanged() {
    var textField = document.getElementById("search");
    var searchText = textField.value;
    var opacity = minOpacity;
    if (searchText == "" || document.activeElement != textField) {
        opacity = 1;
        g_filterText = "";
    } else {
        g_filterText = searchText;
    }

    /* change opacity */
    // TODO: call fade() instead?
    node.style("stroke-opacity", function(o) {
        thisOpacity = o.name.includes(searchText) ? 1 : opacity;
        this.setAttribute('fill-opacity', thisOpacity);
        return thisOpacity;
    });

    text.style("stroke-opacity", function(o) {
        thisOpacity = o.name.includes(searchText) ? 1 : opacity;
        this.setAttribute('fill-opacity', thisOpacity);
        return thisOpacity;
    });
    link.style("stroke-opacity", function(o) {
        return opacity;
    });
}
document.getElementById("search").addEventListener("keyup", searchTextChanged);
document.getElementById("search").addEventListener("focusout", searchTextChanged);
document.getElementById("search").addEventListener("focusin", searchTextChanged);
document.getElementById("select-graph").addEventListener("change", reloadSimulation);



var svg = d3.select("#svg-graph"),
    width = +svg.attr("width"),
    height = +svg.attr("height");


var collisionForce = rectCollide()
    .size(function (d) { return [d.width+10, d.height+6]; });

var boxForce = boundedBox()
    .bounds([[0, 0], [width, height]])
    .size(function (d) { return [d.width, d.height]; });

var simulation = d3.forceSimulation()
    .velocityDecay(0.3) // default: 0.4
    // alpha: initially 1, then reduced at each step, reducing the forces, so
    // that the simulation comes to a stop eventually
    .alphaMin(0.0001) // default: 0.001
    .alphaDecay(0.0428) // default: 0.0228
    //.alphaTarget(1) // enabling this will make sure the simulation never comes
    // to a stop (and the nodes will either keep fighting for their position, or
    // find an equilibrium)
    .force("link", d3.forceLink().id(function(d) { return d.id; })
    .distance(100)//.strength(0.02) // default: 30, 1 / Math.min(count(link.source), count(link.target));
        // distance: desired link distance
//      .iterations(1) // default: 1, greater = increased rigidity
    )
    .force("charge", d3.forceManyBody().strength(-250)) // decrease to make the
           // graph spread more (distance has a similar effect, but affects the
           // leaf nodes more)
    .force('box', boxForce) // keep the nodes inside the visible area
    .force('collision', collisionForce)
    .force("center", d3.forceCenter(width / 2, height / 2));

// SVG elements
var node = null;
var text = null;
var link = null;

function loadSimulation(json_file_name) {

    d3.json(json_file_name, function(error, graph) {
        if (error) throw error;

        // module filtering (does not remove 'orphaned' topics)
        /*
        var ignored_modules = ["mavlink", "commander"];
        for (var i = 0; i < ignored_modules.length; ++i) {
            var module_id = "m_"+ignored_modules[i];
            // links
            for (var j = 0; j < graph.links.length; ++j) {
                if (graph.links[j].source == module_id ||
                    graph.links[j].target == module_id) {
                    graph.links.splice(j, 1);
                    --j;
                }
            }
            // nodes
            for (var j = 0; j < graph.nodes.length; ++j) {
                if (graph.nodes[j].id == module_id) {
                    graph.nodes.splice(j, 1);
                    --j;
                }
            }
        }
        */

        // change style for bidirectional edges
        const edgeMap = new Map();
        var i = 0;
        while(i<graph.links.length) {
            var curr_source = graph.links[i].source;
            var curr_target = graph.links[i].target;
            if (edgeMap.has(curr_target + curr_source) == true) {
                graph.links.splice(i,1);
                graph.links[edgeMap.get(curr_target + curr_source)].style = "dot-dashed";
            }
            else
                edgeMap.set(curr_source + curr_target, i++);
        }

        // explanation for the following syntax: https://bost.ocks.org/mike/join/
        link = svg.append("g")
            .attr("class", "links")
            .selectAll("line")
            .data(graph.links)
            .enter().append("line")
            .attr("stroke-opacity", 0.7)
            .attr("stroke", function(d) { return d.color; })
            .style("stroke-dasharray", function(d) {
                if (d.style == "dashed") return "3, 3";
                if (d.style == "dot-dashed") return "3, 3, 9, 3";
                return "1, 0";
            });

        var g = svg.append("g").selectAll("g").data(graph.nodes).enter().append("g");
        node = g.append("rect")
            // rounded corners (somewhat more expensive to render)
            .attr("rx", function(d) { return d.type == "module" ? 8 : 0; });

        text = g.append("text")
            .attr("class", "labels")
            .style("font-size", "12px")
            .attr("fill", function(d) { return "#fff"; })
            .attr("dy", ".35em")
            .attr("text-anchor", "middle")
            .text(function(d) { return d.name; })
            .on("mouseover", fadeAnimated(minOpacity))
            .on("mouseout", fadeAnimated(1))
            .on("dblclick", openLink);

        var paddingLeftRight = 18; // adjust the padding values depending on font and font size
        var paddingTopBottom = 5;

        svg.selectAll("text").each(function(d, i) {
            var curPaddingLeftRight = paddingLeftRight;
            var curPaddingTopBottom = paddingTopBottom;
            if (graph.nodes[i].type == "module") {
                curPaddingLeftRight *= 1.5;
                curPaddingTopBottom *= 1.5;
            }

            // get bounding box of text field and store it
            graph.nodes[i].width = this.getBBox().width+curPaddingLeftRight;
            graph.nodes[i].height = this.getBBox().height+curPaddingTopBottom;

            graph.nodes[i].vx = 0;
            graph.nodes[i].vy = 0;
        });


        simulation
            .nodes(graph.nodes)
            .on("tick", ticked);

        simulation.force("link")
            .links(graph.links);

        function ticked() {
            link
                .attr("x1", function(d) { return d.source.x; })
                .attr("y1", function(d) { return d.source.y; })
                .attr("x2", function(d) { return d.target.x; })
                .attr("y2", function(d) { return d.target.y; });

            text
                .attr("x", function(d) { return d.x; })
                .attr("y", function(d) { return d.y; });

            svg.selectAll("rect")
                .attr("x", function(d) { return d.x - d.width/2; })
                .attr("y", function(d) { return d.y - d.height/2;  })
                .attr("width", function(d) { return d.width; })
                .attr("height", function(d) { return d.height; })
                .attr("fill", function(d) { return d.color; });
        }

        // open the 'node.url' attribute in a new tab, if it exists
        function openLink(n) {
            if (typeof n.url !== 'undefined') {
                window.open(n.url, '_blank');
            }
        }


        // smooth fade in/out
        var animationTimer = null;
        var currentOpacity = 1;
        var destOpacity = 1;
        function fadeAnimated(opacity) {
            return function(d) {
                if (animationTimer != null)
                    animationTimer.stop();

                destOpacity = opacity;

                animationTimer = d3.interval(function(elapsed) {
                    var newOpacity = currentOpacity + (destOpacity-currentOpacity) * elapsed/300;
                    // check if we overshot the destination opacity
                    if ((currentOpacity - destOpacity) * (newOpacity - destOpacity) < 0) {
                        currentOpacity = destOpacity;
                    } else {
                        currentOpacity = newOpacity;
                    }
                    fade(currentOpacity)(d);
                    if (Math.abs(currentOpacity - destOpacity) < 0.005) {
                        animationTimer.stop();
                        animationTimer = null;
                    }
                }, 30);
            }
        }

        // mouse over functionality: fade the rest of the graph

        var linkedByIndex = {};
        graph.links.forEach(function(d) {
            linkedByIndex[d.source.index + "," + d.target.index] = 1;
        });
        function isConnected(a, b) {
            return linkedByIndex[a.index + "," + b.index] || linkedByIndex[b.index + "," + a.index] || a.index == b.index;
        }
        function fade(opacity) {
            return function(d) {
                /* The graph opacity is using the following behavior:
                 * - no filtering (g_filterText == ""):
                 *   - mouse hovers over a node: the node and it's connected
                 *     nodes are visible, the rest is faded
                 *   - else: all nodes and links are visible
                 * - filtering:
                 *   - all nodes matching the filter are always visible
                 *   - mouse hovers over a node: the connected nodes are visible
                 *   - no hovering: rest of the non-matching nodes are faded
                 *     (and all the links too)
                 */
                var invertedOpacity = (1+minOpacity) - opacity;
                if (g_filterText != "") {
                    // in case of filtering, the default is to fade non-matching
                    // nodes
                    opacity = minOpacity;
                }

                function nodeOpacity(o) {
                    if (g_filterText != "" && o.name.includes(g_filterText)) {
                        thisOpacity = 1; // always visible if filter matches
                    } else if (isConnected(d, o)) {
                        if (g_filterText == "") {
                            thisOpacity = 1; // connected w/o filtering -> show it
                        } else if (d.name.includes(g_filterText)) {
                            thisOpacity = invertedOpacity;
                        } else {
                            thisOpacity = opacity;
                        }
                    } else {
                        thisOpacity = opacity;
                    }

                    this.setAttribute('fill-opacity', thisOpacity);
                    return thisOpacity;
                }

                node.style("stroke-opacity", nodeOpacity);
                text.style("stroke-opacity", nodeOpacity);

                var linkOpacity = opacity;
                var linkOpacityConnected = 1;
                if (g_filterText != "") {
                    if (d.name.includes(g_filterText)) {
                        linkOpacityConnected = invertedOpacity;
                    } else {
                        linkOpacityConnected = minOpacity;
                    }
                    linkOpacity = minOpacity;
                }
                link.style("stroke-opacity", function(o) {
                    return o.source === d || o.target === d ?
                        linkOpacityConnected : linkOpacity;
                });
            };
        }

    });
}

function reloadSimulation(e) {
    json_file_name = e.target.value;
    console.log(json_file_name);
    d3.selectAll("svg > *").remove();
    loadSimulation(json_file_name);
    simulation.alpha(1).restart();
}

/* initial graph */
loadSimulation(default_json_file);


function rectCollide() {
    var nodes, sizes, masses;
    var size = constant([0, 0]);
    var strength = 0.3;
    var iterations = 20;

    function force() {
        var node, size, mass, xi, yi;
        var i = -1;
        while (++i < iterations) { iterate(); }

        function iterate() {
            var j = -1;
            var tree = d3.quadtree(nodes, xCenter, yCenter).visitAfter(prepare);

            while (++j < nodes.length) {
                node = nodes[j];
                size = sizes[j];
                mass = masses[j];
                xi = xCenter(node);
                yi = yCenter(node);

                tree.visit(apply);
            }
        }

        function apply(quad, x0, y0, x1, y1) {
            var data = quad.data;
            var xSize = (size[0] + quad.size[0]) / 2;
            var ySize = (size[1] + quad.size[1]) / 2;
            if (data) {
                if (data.index <= node.index) { return; }

                var x = xi - xCenter(data);
                var y = yi - yCenter(data);
                var xd = Math.abs(x) - xSize;
                var yd = Math.abs(y) - ySize;

                if (xd < 0 && yd < 0) {
                    var l = Math.sqrt(x * x + y * y);
                    var m = masses[data.index] / (mass + masses[data.index]);

                    if (l > 0.000001) {
                        if (xd > yd) {
                            node.vx -= (x *= xd / l * strength) * m;
                            data.vx += x * (1 - m);
                        } else {
                            node.vy -= (y *= yd / l * strength) * m;
                            data.vy += y * (1 - m);
                        }
                    }
                }
            }

            return x0 > xi + xSize || y0 > yi + ySize ||
                x1 < xi - xSize || y1 < yi - ySize;
        }

        function prepare(quad) {
            if (quad.data) {
                quad.size = sizes[quad.data.index];
            } else {
                quad.size = [0, 0];
                var i = -1;
                while (++i < 4) {
                    if (quad[i] && quad[i].size) {
                        quad.size[0] = Math.max(quad.size[0], quad[i].size[0]);
                        quad.size[1] = Math.max(quad.size[1], quad[i].size[1]);
                    }
                }
            }
        }
    }

    function xCenter(d) { return d.x + d.vx; }
    function yCenter(d) { return d.y + d.vy; }

    force.initialize = function (_) {
        sizes = (nodes = _).map(size);
        masses = sizes.map(function (d) { return d[0] * d[1] });
    }

    force.size = function (_) {
        return (arguments.length
            ? (size = typeof _ === 'function' ? _ : constant(_), force)
            : size);
    }

    force.strength = function (_) {
        return (arguments.length ? (strength = +_, force) : strength);
    }

    force.iterations = function (_) {
        return (arguments.length ? (iterations = +_, force) : iterations);
    }

    return force;
}

function boundedBox() {
    var nodes, sizes;
    var bounds;
    var size = constant([0, 0]);

    function force() {
        var node, size;
        var xi, x0, x1, yi, y0, y1;
        var i = -1;
        while (++i < nodes.length) {
            node = nodes[i];
            size = sizes[i];
            xi = node.x + node.vx;
            x0 = bounds[0][0] - (xi - size[0]/2);
            x1 = bounds[1][0] - (xi + size[0]/2);
            yi = node.y + node.vy;
            y0 = bounds[0][1] - (yi - size[1]/2);
            y1 = bounds[1][1] - (yi + size[1]/2);
            if (x0 > 0 || x1 < 0) {
                node.x += node.vx;
                node.vx = -node.vx;
                if (node.vx < x0) { node.x += x0 - node.vx; }
                if (node.vx > x1) { node.x += x1 - node.vx; }
            }
            if (y0 > 0 || y1 < 0) {
                node.y += node.vy;
                node.vy = -node.vy;
                if (node.vy < y0) { node.vy += y0 - node.vy; }
                if (node.vy > y1) { node.vy += y1 - node.vy; }
            }
        }
    }

    force.initialize = function (_) {
        sizes = (nodes = _).map(size);
    }

    force.bounds = function (_) {
        return (arguments.length ? (bounds = _, force) : bounds);
    }

    force.size = function (_) {
        return (arguments.length
            ? (size = typeof _ === 'function' ? _ : constant(_), force)
            : size);
    }

    return force;
}


function constant(_) {
    return function () { return _; }
}

} // initializeGraph()

initializeGraph();
