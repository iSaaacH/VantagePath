# VantagePath Studio

Serve this directory with any static web server:

```sh
python3 -m http.server 4173 --directory editor
```

Then open <http://localhost:4173>. The editor has no package install or build
step. Documents autosave in the browser and can be downloaded as `.vpath`
JSON. C++ export supports the bottom-left corner authoring frame or the official
VEX GPS centre frame.

Run the pure model tests with:

```sh
node --test editor/model.test.mjs
```
