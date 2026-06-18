const puppeteer = require('puppeteer');
const fs = require('fs');
const path = require('path');
const http = require('http');
const https = require('https');
const crypto = require('crypto');
const os = require('os');
const { parse } = require('url');
const busboy = require('busboy');
const process = require('process');

const PORT = 13339;
const DEBUG = false
const DEBUG_TRAJECTORY = false

// Content-addressed cache functionality
function getCacheDir() {
    const cacheDir = path.join(os.homedir(), '.cache', 'rl_tools', 'conta');
    if (!fs.existsSync(cacheDir)) {
        fs.mkdirSync(cacheDir, { recursive: true });
    }
    return cacheDir;
}

async function downloadFile(url, filePath, redirectCount = 0) {
    if (redirectCount > 5) throw new Error('Too many redirects');
    
    return new Promise((resolve, reject) => {
        const request = https.get(url, (response) => {
            console.log(`Response status: ${response.statusCode}`);
            // Handle redirects
            if (response.statusCode >= 300 && response.statusCode < 400 && response.headers.location) {
                console.log(`Redirecting to ${response.headers.location}`);
                return downloadFile(response.headers.location, filePath, redirectCount + 1)
                    .then(resolve).catch(reject);
            }
            
            if (response.statusCode !== 200) {
                return resolve({ status: response.statusCode, contentType: null, data: null });
            }

            const chunks = [];
            response.on('data', chunk => chunks.push(chunk));
            response.on('end', () => {
                const data = Buffer.concat(chunks);
                fs.writeFileSync(filePath, data);
                resolve({ 
                    status: 200, 
                    contentType: response.headers['content-type'] || 'application/octet-stream', 
                    data 
                });
            });
        });
        request.on('error', reject);
    });
}

class Renderer{
    constructor(ui, width, height, model){
        this.ui = ui
        this.width = width
        this.height = height
    }
    async init(){
        this.browser = await puppeteer.launch({headless: "new", args: [
            '--no-sandbox',
            '--disable-setuid-sandbox',
        ], timeout: 0});
        this.page = await this.browser.newPage();
        await this.page.setViewport({ width: this.width, height: this.height });
        await this.page.goto(`http://localhost:${PORT}/`);
    }
    async render(parameters, state, options){
        await this.page.evaluate(async (ui, parameters, state, options) => {
            await window.init(ui, options);
            await window.render_single_frame(parameters, state)
        }, this.ui, parameters, state, options);
        const canvas = await this.page.$('canvas');
        const buffer = await canvas.screenshot({ type: 'png', omitBackground: true});

        await this.browser.close();
        return buffer
    }
    async render_trajectory(parameters, trajectory, options) {
        
        await this.page.evaluate(async (ui, options) => {
            await window.init(ui, options);
        }, this.ui, options);
        const frames = await this.page.evaluate(async (parameters, trajectory) => {
            return await window.render_trajectory(parameters, trajectory)
        }, parameters, trajectory);
        const mean_dt = trajectory.reduce((a, c) => a + c.dt, 0) / trajectory.length;
        const fps = Math.round(1 / mean_dt);
        console.log(`FPS: ${fps}`);
        const buffers = frames.map(f => Buffer.from(f, 'base64'));
        return buffers
    }
    async close(){
        await this.browser.close();
    }
}



const server = http.createServer((req, res) => {
    console.log(req.url)
    const parsedUrl = parse(req.url, true);
    const pathname = parsedUrl.pathname;

    if (pathname === '/' || pathname === '/' + "puppeteer.html"){
        res.writeHead(200, { 'Content-Type': 'text/html' });
        const HTML_CONTENT = fs.readFileSync(path.join(__dirname, "puppeteer.html"), 'utf8');
        res.end(HTML_CONTENT);
    } else if (req.url.startsWith('/lib/')) {
        const filePath = path.join(__dirname, req.url);
        fs.readFile(filePath, (err, data) => {
            if(err){
                res.writeHead(404);
                res.end('File not found');
                return;
            }

            const ext = path.extname(filePath).toLowerCase();
            let contentType = 'application/octet-stream';
            if (ext === '.js') contentType = 'application/javascript';
            else if (ext === '.wasm') contentType = 'application/wasm';
            else if (ext === '.json') contentType = 'application/json';

            res.writeHead(200, { 'Content-Type': contentType });
            res.end(data);
        })
    } else if (req.method === 'GET' && req.url === '/data.json') {
        const DATA_CONTENT = fs.readFileSync(path.join(__dirname, "data.json"), 'utf8')
        const DATA = JSON.parse(DATA_CONTENT)
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(DATA));
    } else if (req.method === 'GET' && req.url === '/ui.js') {
        res.writeHead(200, { 'Content-Type': 'application/javascript' });
        const UI = fs.readFileSync(path.join(__dirname, "ui.js"), 'utf8');
        res.end(UI);
    } else if (req.method === 'GET' && pathname.startsWith('/conta/')) {
        // Handle content-addressed asset requests
        const sha1Hash = pathname.split('/conta/')[1];
        
        // Validate hash format
        if (!sha1Hash || sha1Hash.length !== 40 || !/^[0-9a-fA-F]+$/.test(sha1Hash)) {
            res.writeHead(400, { 'Content-Type': 'text/plain' });
            res.end('Invalid SHA-1 hash format');
            return;
        }
        
        const cacheFile = path.join(getCacheDir(), sha1Hash);
        
        // Return cached file if it exists
        if (fs.existsSync(cacheFile)) {
            res.writeHead(200, { 'Content-Type': 'application/octet-stream' });
            fs.createReadStream(cacheFile).pipe(res);
            return;
        }
        
        // Download and verify
        const url = `https://rl-tools.github.io/conta-data/data/${sha1Hash}`;
        console.log(`Downloading ${url}`);
        
        downloadFile(url, cacheFile).then(result => {
            if (result.status === 404) return res.writeHead(404) || res.end('Asset not found');
            if (result.status !== 200) return res.writeHead(502) || res.end(`Failed to fetch asset: HTTP ${result.status}`);
            
            // Verify hash
            const computedHash = crypto.createHash('sha1').update(result.data).digest('hex');
            if (computedHash !== sha1Hash.toLowerCase()) {
                fs.existsSync(cacheFile) && fs.unlinkSync(cacheFile);
                return res.writeHead(422) || res.end('Hash mismatch: downloaded content does not match requested hash');
            }
            
            res.writeHead(200, { 'Content-Type': result.contentType, 'Content-Length': result.data.length });
            res.end(result.data);
            
        }).catch(err => {
            console.error('Download error:', err);
            res.writeHead(502, { 'Content-Type': 'text/plain' });
            res.end(`Network error: ${err.message}`);
        });
    } else if (req.method === 'POST' && (req.url === '/render_trajectory' || req.url === '/render')) {
        // input: {parameters, trajectory: [{state, action, dt, ...}]}, ui: ui.js
        const bb = busboy({ headers: req.headers });
        const files = {};
        const fields = {};
        bb.on('file', (name, file, info) => {
            const { filename, encoding, mimeType } = info;
            const combined_data = [];
            console.log(
                `File [${name}]: filename: %j, encoding: %j, mimeType: %j`,
                filename,
                encoding,
                mimeType
            );
            file.on('data', (data) => {
                console.log(`File [${name}] got ${data.length} bytes`);
                combined_data.push(data);
            }).on('close', () => {
                console.log(`File [${name}] done`);
                files[name] = Buffer.concat(combined_data);
            });
        });
        bb.on('field', (name, val, info) => {
            console.log(`Field [${name}]: value: %j`, val);
            fields[name] = val;
        });
        bb.on('close', async () => {
            console.log('Done parsing form!');
            try {
                const payload = JSON.parse(files["data"]);
                const ui = files["ui"].toString('utf8');
                const options = files["options"] ? JSON.parse(files["options"].toString('utf8')) : {};
                const renderer = new Renderer(ui, parseInt(fields["width"]), parseInt(fields["height"]));
                await renderer.init();
                let frames = null;
                if(req.url === '/render'){
                    const frame = await renderer.render(payload.parameters, payload.step, options)
                    await renderer.close();

                    res.writeHead(200, {
                        'Content-Type': 'image/png',
                        'Content-Length': frame.length,
                    });
                    res.end(frame);
                }
                else{
                    const yazl = require('yazl');
                    frames = await renderer.render_trajectory(payload.parameters, payload.trajectory, options)
                    await renderer.close();

                    const zipfile = new yazl.ZipFile();
                    frames.forEach((frame, i) => {
                        const name = `frame_${i.toString().padStart(5, '0')}.png`;
                        zipfile.addBuffer(frame, name);
                    });

                    res.writeHead(200, {
                        'Content-Type': 'application/zip',
                        'Content-Disposition': 'attachment; filename="frames.zip"'
                    });
                    zipfile.outputStream.pipe(res);
                    zipfile.end();
                }
            } catch (err) {
                console.error('Render error:', err);
                res.writeHead(500);
                res.end('Failed to render canvas.');
            }
        });
        req.pipe(bb);
    } else {
        res.writeHead(404);
        res.end('Not Found');
    }
});


async function main_debug(){
    await new Promise(resolve => server.listen(PORT, resolve));
    const DATA_CONTENT = fs.readFileSync(path.join(__dirname, "data.json"), 'utf8')
    const DATA = JSON.parse(DATA_CONTENT)
    const UI = fs.readFileSync(path.join(__dirname, "ui.js"), 'utf8');

    const renderer = new Renderer(UI, 2000, 2000);
    await renderer.init();

    if(DEBUG_TRAJECTORY){
        frames = await renderer.render_trajectory(DATA[0].parameters, DATA[0].trajectory, {frame_counter: false})
        await renderer.close();

        const outputDir = path.join(__dirname, 'canvas_frames');
        if (!fs.existsSync(outputDir)) {
            fs.mkdirSync(outputDir, { recursive: true });
        }
        const file_promises = frames.map((frame, i) => {
            const outputPath = path.join(outputDir, `frame_${i.toString().padStart(5, '0')}.png`);
            console.log(`Saved frame ${i} to ${outputPath}`);
            return fs.promises.writeFile(outputPath, frame);
        })
        await Promise.all(file_promises);
    }
    else{
        const buffer = await renderer.render(DATA[0].parameters, DATA[0].trajectory[DATA[0].trajectory.length - 1], {camera_position: [0, 0, 2]});
        const outputPath = path.join(__dirname, 'output.png');
        fs.writeFileSync(outputPath, buffer);
        console.log(`Screenshot saved to ${outputPath}`);
    }
    await new Promise(resolve => setTimeout(resolve, 100000000));
    server.close(() => console.log('Server closed.'));
}
async function main(){
    server.listen(PORT);
}
process.on('SIGINT', () => {
    console.log('Shutting down...');
    process.exit(0);
});




if (DEBUG) {
    main_debug()
}
else{
    main()
}

