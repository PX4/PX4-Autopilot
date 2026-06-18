import {renderDOM} from "./render.js";
import {main} from "./main.js";

window.addEventListener('DOMContentLoaded', ()=>{
    renderDOM(document.body);
})
window.addEventListener('load', ()=>{
    main();
})
