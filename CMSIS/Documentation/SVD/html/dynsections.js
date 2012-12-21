var showTriggers = new Array();

function registerShow(sectId,showFunc) {
  showTriggers[sectId] = showFunc;
}

function hasClass(ele,cls) {
  return ele.className.match(new RegExp('(\\s|^)'+cls+'(\\s|$)'));
}

function addClass(ele,cls) {
  if (!this.hasClass(ele,cls)) ele.className += " "+cls;
}

function removeClass(ele,cls) {
  if (hasClass(ele,cls)) {
    var reg = new RegExp('(\\s|^)'+cls+'(\\s|$)');
    ele.className=ele.className.replace(reg,' ');
  }
}

function toggleVisibility(linkObj) {
 var base = linkObj.getAttribute('id');
 var summary = document.getElementById(base + '-summary');
 var content = document.getElementById(base + '-content');
 var trigger = document.getElementById(base + '-trigger');
 if ( hasClass(linkObj,'closed') ) {
   summary.style.display = 'none';
   content.style.display = 'block';
   trigger.src = trigger.src.substring(0,trigger.src.length-10)+'open.png';
   removeClass(linkObj,'closed');
   addClass(linkObj,'opened');
   if (showTriggers[base]) { showTriggers[base](); }
 } else if ( hasClass(linkObj,'opened') ) {
   summary.style.display = 'block';
   content.style.display = 'none';
   trigger.src = trigger.src.substring(0,trigger.src.length-8)+'closed.png';
   removeClass(linkObj,'opened');
   addClass(linkObj,'closed');
 }
 return false;
}
