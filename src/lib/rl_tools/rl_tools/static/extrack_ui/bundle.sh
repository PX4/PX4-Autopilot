npm install esbuild --save-dev
npx esbuild dependencies/dependencies.js --bundle --minify --format=esm --outfile=lib/dependencies.js
npx esbuild dependencies/chart.js --bundle --minify --format=esm --outfile=lib/chart.js
npx esbuild dependencies/three.js --bundle --minify --format=esm --outfile=lib/three.js
mkdir -p lib/ace
cp node_modules/ace-builds/src-noconflict/ace.js lib/ace
cp node_modules/ace-builds/src-noconflict/keybinding-vim.js lib/ace
cp node_modules/ace-builds/src-noconflict/mode-javascript.js lib/ace
cp node_modules/ace-builds/src-noconflict/theme-tomorrow.js lib/ace
cp node_modules/ace-builds/src-noconflict/worker-javascript.js lib/ace
