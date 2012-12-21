var NAVTREE =
[
  [ "CMSIS-CORE", "index.html", [
    [ "Overview", "index.html", null ],
    [ "Usage and Description", "pages.html", [
      [ "Using CMSIS in Embedded Applications", "_using_pg.html", [
        [ "Using CMSIS with generic ARM Processors", "_using__a_r_m_pg.html", null ]
      ] ],
      [ "Template Files", "_templates_pg.html", [
        [ "Startup File startup_<device>.s", "startup_s_pg.html", null ],
        [ "System Configuration Files system_<device>.c and system_<device>.h", "system_c_pg.html", null ],
        [ "Device Header File <device.h>", "device_h_pg.html", null ]
      ] ],
      [ "MISRA-C:2004 Compliance Exceptions", "_c_o_r_e__m_i_s_r_a__exceptions_pg.html", null ],
      [ "Register Mapping", "_reg_map_pg.html", null ]
    ] ],
    [ "Reference", "modules.html", [
      [ "Peripheral Access", "group__peripheral__gr.html", null ],
      [ "System and Clock Configuration", "group__system__init__gr.html", null ],
      [ "Interrupts and Exceptions (NVIC)", "group___n_v_i_c__gr.html", null ],
      [ "Core Register Access", "group___core___register__gr.html", null ],
      [ "Intrinsic Functions for CPU Instructions", "group__intrinsic___c_p_u__gr.html", null ],
      [ "Intrinsic Functions for SIMD Instructions [only Cortex-M4]", "group__intrinsic___s_i_m_d__gr.html", null ],
      [ "Systick Timer (SYSTICK)", "group___sys_tick__gr.html", null ],
      [ "Debug Access", "group___i_t_m___debug__gr.html", null ]
    ] ],
    [ "Data Structures", "annotated.html", [
      [ "APSR_Type", "union_a_p_s_r___type.html", null ],
      [ "CONTROL_Type", "union_c_o_n_t_r_o_l___type.html", null ],
      [ "CoreDebug_Type", "struct_core_debug___type.html", null ],
      [ "DWT_Type", "struct_d_w_t___type.html", null ],
      [ "FPU_Type", "struct_f_p_u___type.html", null ],
      [ "IPSR_Type", "union_i_p_s_r___type.html", null ],
      [ "ITM_Type", "struct_i_t_m___type.html", null ],
      [ "MPU_Type", "struct_m_p_u___type.html", null ],
      [ "NVIC_Type", "struct_n_v_i_c___type.html", null ],
      [ "SCB_Type", "struct_s_c_b___type.html", null ],
      [ "SCnSCB_Type", "struct_s_cn_s_c_b___type.html", null ],
      [ "SysTick_Type", "struct_sys_tick___type.html", null ],
      [ "TPI_Type", "struct_t_p_i___type.html", null ],
      [ "xPSR_Type", "unionx_p_s_r___type.html", null ]
    ] ],
    [ "Data Structure Index", "classes.html", null ],
    [ "Data Fields", "functions.html", null ],
    [ "Index", "globals.html", null ]
  ] ]
];

function createIndent(o,domNode,node,level)
{
  if (node.parentNode && node.parentNode.parentNode)
  {
    createIndent(o,domNode,node.parentNode,level+1);
  }
  var imgNode = document.createElement("img");
  if (level==0 && node.childrenData)
  {
    node.plus_img = imgNode;
    node.expandToggle = document.createElement("a");
    node.expandToggle.href = "javascript:void(0)";
    node.expandToggle.onclick = function() 
    {
      if (node.expanded) 
      {
        $(node.getChildrenUL()).slideUp("fast");
        if (node.isLast)
        {
          node.plus_img.src = node.relpath+"ftv2plastnode.png";
        }
        else
        {
          node.plus_img.src = node.relpath+"ftv2pnode.png";
        }
        node.expanded = false;
      } 
      else 
      {
        expandNode(o, node, false);
      }
    }
    node.expandToggle.appendChild(imgNode);
    domNode.appendChild(node.expandToggle);
  }
  else
  {
    domNode.appendChild(imgNode);
  }
  if (level==0)
  {
    if (node.isLast)
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2plastnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2lastnode.png";
        domNode.appendChild(imgNode);
      }
    }
    else
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2pnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2node.png";
        domNode.appendChild(imgNode);
      }
    }
  }
  else
  {
    if (node.isLast)
    {
      imgNode.src = node.relpath+"ftv2blank.png";
    }
    else
    {
      imgNode.src = node.relpath+"ftv2vertline.png";
    }
  }
  imgNode.border = "0";
}

function newNode(o, po, text, link, childrenData, lastNode)
{
  var node = new Object();
  node.children = Array();
  node.childrenData = childrenData;
  node.depth = po.depth + 1;
  node.relpath = po.relpath;
  node.isLast = lastNode;

  node.li = document.createElement("li");
  po.getChildrenUL().appendChild(node.li);
  node.parentNode = po;

  node.itemDiv = document.createElement("div");
  node.itemDiv.className = "item";

  node.labelSpan = document.createElement("span");
  node.labelSpan.className = "label";

  createIndent(o,node.itemDiv,node,0);
  node.itemDiv.appendChild(node.labelSpan);
  node.li.appendChild(node.itemDiv);

  var a = document.createElement("a");
  node.labelSpan.appendChild(a);
  node.label = document.createTextNode(text);
  a.appendChild(node.label);
  if (link) 
  {
    a.href = node.relpath+link;
  } 
  else 
  {
    if (childrenData != null) 
    {
      a.className = "nolink";
      a.href = "javascript:void(0)";
      a.onclick = node.expandToggle.onclick;
      node.expanded = false;
    }
  }

  node.childrenUL = null;
  node.getChildrenUL = function() 
  {
    if (!node.childrenUL) 
    {
      node.childrenUL = document.createElement("ul");
      node.childrenUL.className = "children_ul";
      node.childrenUL.style.display = "none";
      node.li.appendChild(node.childrenUL);
    }
    return node.childrenUL;
  };

  return node;
}

function showRoot()
{
  var headerHeight = $("#top").height();
  var footerHeight = $("#nav-path").height();
  var windowHeight = $(window).height() - headerHeight - footerHeight;
  navtree.scrollTo('#selected',0,{offset:-windowHeight/2});
}

function expandNode(o, node, imm)
{
  if (node.childrenData && !node.expanded) 
  {
    if (!node.childrenVisited) 
    {
      getNode(o, node);
    }
    if (imm)
    {
      $(node.getChildrenUL()).show();
    } 
    else 
    {
      $(node.getChildrenUL()).slideDown("fast",showRoot);
    }
    if (node.isLast)
    {
      node.plus_img.src = node.relpath+"ftv2mlastnode.png";
    }
    else
    {
      node.plus_img.src = node.relpath+"ftv2mnode.png";
    }
    node.expanded = true;
  }
}

function getNode(o, po)
{
  po.childrenVisited = true;
  var l = po.childrenData.length-1;
  for (var i in po.childrenData) 
  {
    var nodeData = po.childrenData[i];
    po.children[i] = newNode(o, po, nodeData[0], nodeData[1], nodeData[2],
        i==l);
  }
}

function findNavTreePage(url, data)
{
  var nodes = data;
  var result = null;
  for (var i in nodes) 
  {
    var d = nodes[i];
    if (d[1] == url) 
    {
      return new Array(i);
    }
    else if (d[2] != null) // array of children
    {
      result = findNavTreePage(url, d[2]);
      if (result != null) 
      {
        return (new Array(i).concat(result));
      }
    }
  }
  return null;
}

function initNavTree(toroot,relpath)
{
  var o = new Object();
  o.toroot = toroot;
  o.node = new Object();
  o.node.li = document.getElementById("nav-tree-contents");
  o.node.childrenData = NAVTREE;
  o.node.children = new Array();
  o.node.childrenUL = document.createElement("ul");
  o.node.getChildrenUL = function() { return o.node.childrenUL; };
  o.node.li.appendChild(o.node.childrenUL);
  o.node.depth = 0;
  o.node.relpath = relpath;

  getNode(o, o.node);

  o.breadcrumbs = findNavTreePage(toroot, NAVTREE);
  if (o.breadcrumbs == null)
  {
    o.breadcrumbs = findNavTreePage("index.html",NAVTREE);
  }
  if (o.breadcrumbs != null && o.breadcrumbs.length>0)
  {
    var p = o.node;
    for (var i in o.breadcrumbs) 
    {
      var j = o.breadcrumbs[i];
      p = p.children[j];
      expandNode(o,p,true);
    }
    p.itemDiv.className = p.itemDiv.className + " selected";
    p.itemDiv.id = "selected";
    $(window).load(showRoot);
  }
}

