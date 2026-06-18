VERSION=r156
mkdir lib
mkdir -p lib/gltf-loader
mkdir -p lib/utils
wget https://github.com/mrdoob/three.js/raw/$VERSION/build/three.module.js -O lib/three.module.js
wget https://github.com/mrdoob/three.js/raw/$VERSION/examples/jsm/controls/OrbitControls.js -O lib/OrbitControls.js
wget https://github.com/mrdoob/three.js/raw/$VERSION/examples/jsm/loaders/GLTFLoader.js -O lib/gltf-loader/GLTFLoader.js
wget https://github.com/mrdoob/three.js/raw/$VERSION/examples/jsm/utils/BufferGeometryUtils.js -O lib/utils/BufferGeometryUtils.js

