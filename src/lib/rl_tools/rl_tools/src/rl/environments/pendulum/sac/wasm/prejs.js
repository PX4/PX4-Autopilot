

console.log("prejs")
Module.locateFile = function (path) {
    if (!path.startsWith('http') && !path.startsWith('/')) {
        return './build/' + path;
    }
    return path;
}
