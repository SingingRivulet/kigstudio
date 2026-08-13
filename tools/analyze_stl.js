const fs = require('fs');
const path = process.argv[2];
const text = fs.readFileSync(path, 'latin1');
const lines = text.split(/\r?\n/);

const verts = [];      // raw vertices
const tris = [];       // [i0,i1,i2] into verts (unwelded)
let cur = [];
for (const ln of lines) {
    const t = ln.trim();
    if (t.startsWith('vertex')) {
        const p = t.split(/\s+/);
        cur.push([parseFloat(p[1]), parseFloat(p[2]), parseFloat(p[3])]);
        if (cur.length === 3) { tris.push(cur); cur = []; }
    }
}
console.log('triangles:', tris.length);

// weld with tolerance q
function analyze(q) {
    const map = new Map();
    const wv = [];
    const idx = tris.map(tri => tri.map(v => {
        const key = Math.round(v[0]/q) + ',' + Math.round(v[1]/q) + ',' + Math.round(v[2]/q);
        let i = map.get(key);
        if (i === undefined) { i = wv.length; wv.push(v); map.set(key, i); }
        return i;
    }));
    const edgeCount = new Map();
    const edgeTri = new Map();
    idx.forEach((t, ti) => {
        for (let e = 0; e < 3; e++) {
            let a = t[e], b = t[(e+1)%3];
            if (a === b) continue;
            const k = a < b ? a+'_'+b : b+'_'+a;
            edgeCount.set(k, (edgeCount.get(k)||0)+1);
            if (!edgeTri.has(k)) edgeTri.set(k, []);
            edgeTri.get(k).push(ti);
        }
    });
    let boundary = [], nonman = [], degenerate = 0;
    for (const t of idx) if (t[0]===t[1]||t[1]===t[2]||t[0]===t[2]) degenerate++;
    for (const [k,c] of edgeCount) {
        if (c === 1) boundary.push(k);
        else if (c > 2) nonman.push([k,c]);
    }
    return { wv, idx, boundary, nonman, degenerate, edgeTri };
}

for (const q of [1e-6, 1e-5, 1e-4]) {
    const r = analyze(q);
    console.log(`\n=== weld tol ${q} ===`);
    console.log('welded verts:', r.wv.length, 'degenerate tris:', r.degenerate);
    console.log('boundary edges:', r.boundary.length, 'non-manifold edges(>2):', r.nonman.length);
    if (q === 1e-6) {
        // detail of boundary edges: length and position
        for (const k of r.boundary.slice(0, 30)) {
            const [a,b] = k.split('_').map(Number);
            const A = r.wv[a], B = r.wv[b];
            const len = Math.hypot(A[0]-B[0], A[1]-B[1], A[2]-B[2]);
            console.log(`  edge len=${len.toExponential(2)}  A=(${A.map(x=>x.toFixed(4))}) B=(${B.map(x=>x.toFixed(4))}) tris=${r.edgeTri.get(k).join(',')}`);
        }
        if (r.nonman.length) {
            console.log('  non-manifold edges detail:');
            for (const [k,c] of r.nonman.slice(0, 20)) {
                const [a,b] = k.split('_').map(Number);
                const A = r.wv[a], B = r.wv[b];
                console.log(`    count=${c} A=(${A.map(x=>x.toFixed(4))}) B=(${B.map(x=>x.toFixed(4))}) tris=${r.edgeTri.get(k).join(',')}`);
            }
        }
    }
}
