# 4-Bar Linkage Synthesis Tool

A pure client-side four-bar linkage synthesis tool. No server, no build step, no npm.

## File Structure

```
project/
├── index.html              ← Main HTML shell (zero embedded JS)
├── css/
│   └── style.css           ← All styles (extracted from original front.html)
└── js/
    ├── geometry.js         ← Math helpers + kinematic engine (ES6 module)
    ├── construction.js     ← Linkage synthesis logic (ES6 module)
    ├── canvas.js           ← Pure stateless canvas drawing functions (ES6 module)
    ├── autosolve.worker.js ← Web Worker for brute-force parameter search
    └── main.js             ← App entry point, state, events (ES6 module)
```

## How to Run

> **ES6 modules require a local HTTP server** — you cannot simply open `index.html` as a `file://` URL or imports will be blocked by the browser's CORS policy.

### Option 1 — VS Code Live Server (recommended)
1. Install the [Live Server](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer) extension
2. Right-click `index.html` → **Open with Live Server**

### Option 2 — Python built-in server
```bash
cd project/
python -m http.server 8080
```
Then open `http://localhost:8080` in Chrome.

### Option 3 — Node.js `serve`
```bash
npx serve project/
```

## Architecture Notes

| File | Origin |
|------|--------|
| `geometry.js` | Port of all math functions from `app.py` |
| `construction.js` | Port of the `/calculate` Flask endpoint (`run_synthesis_core`) |
| `autosolve.worker.js` | Port of the `/autosolve` + `/progress` + `/cancel` Flask endpoints, runs in a Web Worker thread |
| `canvas.js` | All drawing code extracted from `front.html` |
| `main.js` | All UI/state/event logic from `front.html`, `fetch()` calls replaced with direct JS calls |

## No Backend Needed

All computation that previously ran in Python (Flask) now runs entirely in the browser:
- **Kinematics** → `computeFullState()` in `construction.js`  
- **Auto-solve search** → `autosolve.worker.js` Web Worker (runs on a separate thread so the UI stays responsive)
- **Cancel** → `worker.terminate()` in `main.js`
