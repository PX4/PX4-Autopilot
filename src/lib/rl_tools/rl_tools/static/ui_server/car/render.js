function createAndAppendElement(tag, attributes, parent) {
    const element = document.createElement(tag);
    Object.keys(attributes).forEach(attr => element[attr] = attributes[attr]);
    parent.appendChild(element);
    return element;
}


function renderCanvas(aspectRatioContainerContainer){
    const aspectRatioContainer = createAndAppendElement('div', {id: 'car-aspectRatioContainer'}, aspectRatioContainerContainer);
    const canvasContainerInner = createAndAppendElement('div', {id: 'car-canvasContainerInner'}, aspectRatioContainer);
    createAndAppendElement('canvas', {id: 'car-drawingCanvas', width: '100', height: '100'}, canvasContainerInner);
    const trainingOverlay = document.createElement("div")
    trainingOverlay.id = 'car-drawingCanvasTrainingOverlay'
    const trainingOverlayText = document.createElement("div")
    trainingOverlayText.id = 'car-drawingCanvasTrainingOverlayText'
    trainingOverlayText.textContent = 'Training...'
    trainingOverlay.style.display = 'none'
    trainingOverlay.appendChild(trainingOverlayText)
    canvasContainerInner.appendChild(trainingOverlay)
}

function renderCanvasContainer(canvasContainer){
    const aspectRatioContainerContainerContainer = createAndAppendElement('div', {id: 'car-aspectRatioContainerContainerContainer'}, canvasContainer);
    const aspectRatioContainerContainer = createAndAppendElement('div', {id: 'car-aspectRatioContainerContainer'}, aspectRatioContainerContainerContainer);
    renderCanvas(aspectRatioContainerContainer)
}

function renderControlInputs(controlContainer){
    createAndAppendElement('input', {type: 'button', className: 'car-input', id: 'car-resetTrackButton', value: 'Reset Track', style: 'display: none;'}, controlContainer);
    createAndAppendElement('input', {type: 'button', className: 'car-input', id: 'car-saveTrackButton', value: 'Save Track', style: 'display: none;'}, controlContainer);
    createAndAppendElement('input', {type: 'button', className: 'car-input', id: 'car-playButton', value: 'Play Interactively', style: 'display: none;'}, controlContainer);
    createAndAppendElement('input', {type: 'button', className: 'car-input', id: 'car-trainButton', value: 'Start Training', style: 'display: none;'}, controlContainer);
    const label = createAndAppendElement('label', {id: 'car-playbackSpeedCheckboxLabel', className: 'checkbox-label', style: 'display: none;'}, controlContainer);
    createAndAppendElement('input', {type: 'checkbox', id: 'car-playbackSpeedCheckbox', className: 'checkbox-input'}, label);
    label.appendChild(createAndAppendElement('span', {className: 'checkbox-custom'}, label))
    label.appendChild(document.createTextNode(' Realtime'));
}

function renderMessageLabels(messageContainer){
    createAndAppendElement('div', {id: 'car-loadingLabel', textContent: 'Loading...'}, messageContainer);
    createAndAppendElement('div', {id: 'car-drawLabel', textContent: 'Draw track onto the canvas!', style: 'display: none;'}, messageContainer);
    createAndAppendElement('div', {id: 'car-playLabel', textContent: 'Use arrow keys to drive the car!', style: 'display: none;'}, messageContainer);
    createAndAppendElement('div', {id: 'car-trainLabel', textContent: 'Training (in the background)...', style: 'display: none;'}, messageContainer);

}

function renderDOM(parentElement) {
    const messageContainer = createAndAppendElement('div', {id: 'car-messageContainer'}, parentElement);
    renderMessageLabels(messageContainer);
    const controlContainer = createAndAppendElement('div', {id: 'car-controlContainer'}, parentElement);
    renderControlInputs(controlContainer);
    const canvasContainer = createAndAppendElement('div', {id: 'car-canvasContainer', style: 'display: none;'}, parentElement);
    renderCanvasContainer(canvasContainer);
}


export {
    renderDOM,
    renderCanvas,
    renderCanvasContainer,
    renderControlInputs,
    renderMessageLabels
};