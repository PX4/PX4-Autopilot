export class DynamicFileSystem {
    constructor(base_path) {
        this.base_path = base_path;
    }

    async loadTree() {
        // First, discover all directories
        const allDirs = new Set();
        const pendingDirs = [""];
        const dirToEntries = new Map();

        while (pendingDirs.length > 0) {
            const currentPath = pendingDirs.shift();
            allDirs.add(currentPath);

            const html = await this.fetchDirectory(currentPath);
            const entries = this.parseDirectoryHtml(html);
            dirToEntries.set(currentPath, entries);

            // Add new directories to pending list
            for (const { name, isDirectory } of entries) {
                if (isDirectory) {
                    const childPath = currentPath + name + "/";
                    if (!allDirs.has(childPath)) {
                        pendingDirs.push(childPath);
                    }
                }
            }
        }

        // Now fetch all directories in parallel
        const dirContentPromises = Array.from(allDirs).map(async dir => ({
            path: dir,
            entries: dirToEntries.get(dir)
        }));

        const allDirContents = await Promise.all(dirContentPromises);

        // Build the tree from our complete directory listing
        return this.buildTree("", allDirContents);
    }

    buildTree(path, allDirContents) {
        const node = {
            children: {},
            path: path
        };

        const dirContent = allDirContents.find(d => d.path === path);
        if (!dirContent) return node;

        for (const { name, isDirectory } of dirContent.entries) {
            if (isDirectory) {
                const childPath = path + name + "/";
                node.children[name] = this.buildTree(childPath, allDirContents);
            } else {
                node.children[name] = this.base_path + path + name;
            }
        }

        return node;
    }

    async fetchDirectory(path) {
        const url = this.base_path + path;
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        return await response.text();
    }

    parseDirectoryHtml(html) {
        const parser = new DOMParser();
        const doc = parser.parseFromString(html, 'text/html');
        const links = Array.from(doc.getElementsByTagName('a'));

        return links
            .map(link => ({
                name: link.textContent,
                isDirectory: link.textContent.endsWith('/')
            }))
            .filter(({name}) => name !== '../' && name.length > 0)
            .map(({name, isDirectory}) => ({
                name: isDirectory ? name.slice(0, -1) : name,
                isDirectory
            }));
    }

    normalize(path) {
        return (new URL(path, window.location.origin)).pathname.substring(1);
    }

    // For compatibility with existing interface
    addNode(node, path, relativePath, fullPath) {
        throw new Error('addNode is not used in DynamicFileSystem');
    }

    parsePath(tree, path) {
        throw new Error('parsePath is not used in DynamicFileSystem');
    }

    parse(index) {
        throw new Error('parse is not used in DynamicFileSystem');
    }
}

