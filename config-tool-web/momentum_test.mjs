import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

// Exercise the real configurator handlers with a small WebHID/DOM substitute.
const nodes = new Map();
const node = id => {
    if (!nodes.has(id)) nodes.set(id, {
        id, value: '', checked: false, files: [], style: {}, classList: { add() {}, remove() {} },
        addEventListener() {}, click() {}, setAttribute(name, value) { this[name] = value; },
    });
    return nodes.get(id);
};
let download;
const document = {
    addEventListener() {}, getElementById: node,
    createElement() { download = node('download_link'); return download; },
    body: { appendChild() {}, removeChild() {} },
};
const context = vm.createContext({ document, structuredClone, setTimeout: () => 1,
    clearTimeout() {}, FileReader: class {
        readAsText(file) { this.onload({ target: { result: file.text } }); }
    },
});
const source = fs.readFileSync(new URL('./code.js', import.meta.url), 'utf8')
    .replace(/^import .*;\n/gm, '');
vm.runInContext(source, context);
vm.runInContext(`
    set_config_ui_state = () => {};
    set_mappings_ui_state = () => {};
    set_macros_ui_state = () => {};
    set_expressions_ui_state = () => {};
    set_quirks_ui_state = () => {};
    setup_usages_modals = () => {};
    validate_ui_expressions = () => {};
    switch_to_mappings_tab = () => {};
    clear_error = () => {};
    display_error = e => { throw e; };
    device = {};
    config.mappings = [];
`, context);
const evaluate = expression => vm.runInContext(expression, context);

// Older JSON imports get disabled defaults; export retains all three fields.
evaluate(`config.version = 27; delete config.tps43_tuning.scroll_momentum_enabled;
    delete config.tps43_tuning.scroll_momentum_launch_strength_percent;
    delete config.tps43_tuning.scroll_momentum_half_life_ms; set_ui_state();`);
assert.deepEqual(JSON.parse(JSON.stringify(evaluate(`[
    config.version, config.tps43_tuning.scroll_momentum_enabled,
    config.tps43_tuning.scroll_momentum_launch_strength_percent,
    config.tps43_tuning.scroll_momentum_half_life_ms]`))), [28, false, 50, 100]);
evaluate(`config.tps43_tuning.scroll_momentum_enabled = true;
    config.tps43_tuning.scroll_momentum_launch_strength_percent = 75;
    config.tps43_tuning.scroll_momentum_half_life_ms = 150;
    download_json();`);
const exported = JSON.parse(decodeURIComponent(download.href.split(',')[1]));
assert.equal(exported.tps43_tuning.scroll_momentum_half_life_ms, 150);
const olderExport = structuredClone(exported);
olderExport.version = 27;
delete olderExport.tps43_tuning.scroll_momentum_enabled;
delete olderExport.tps43_tuning.scroll_momentum_launch_strength_percent;
delete olderExport.tps43_tuning.scroll_momentum_half_life_ms;
node('file_input').files = [{ text: JSON.stringify(olderExport) }];
evaluate('file_uploaded()');
assert.deepEqual(Array.from(evaluate(`[
    config.tps43_tuning.scroll_momentum_enabled,
    config.tps43_tuning.scroll_momentum_launch_strength_percent,
    config.tps43_tuning.scroll_momentum_half_life_ms]`)), [false, 50, 100]);
node('file_input').files = [{ text: JSON.stringify(exported) }];
evaluate('file_uploaded()');
assert.equal(evaluate('config.tps43_tuning.scroll_momentum_launch_strength_percent'), 75);

// Mock feature reports and verify the actual load and save command payloads.
let lastCommand;
const commands = [];
context.featureSend = async (command, fields) => {
    lastCommand = command;
    commands.push([command, fields]);
};
context.featureRead = async () => {
    const table = {
        3: [28, 0, 255, 1000000, 0, 0, 0, 0, 200000, 5, 0, 0, 0],
        26: [200, 100, 20, 2, 128, 4, 50, 2],
        29: [1, 500, 2000, 20, 10, 0],
        31: [0, 200, 1000, 200, 50],
        33: [10, 1], 35: [1, 8, 2], 37: [1, 75, 150],
        17: [0], 21: [0], 25: [0], 7: [1],
    };
    return table[lastCommand] ?? [0];
};
vm.runInContext('send_feature_command = featureSend; read_config_feature = featureRead;', context);
await evaluate('load_from_device()');
assert.equal(evaluate('config.tps43_tuning.scroll_momentum_enabled'), true);
assert.equal(evaluate('config.tps43_tuning.scroll_momentum_half_life_ms'), 150);
commands.length = 0;
await evaluate('save_to_device()');
const momentumWrite = commands.find(([command]) => command === 38);
assert.ok(momentumWrite, 'save must write momentum controls');
assert.deepEqual(Array.from(momentumWrite[1], ([, value]) => value), [1, 75, 150]);
context.invalidFeatureRead = async () => lastCommand === 37 ? [255, 65535, 65535] : context.featureRead();
vm.runInContext('read_config_feature = invalidFeatureRead;', context);
await assert.rejects(evaluate('load_from_device()'), /Invalid scroll momentum readback/);
console.log('Configurator load/save and JSON import/export momentum checks passed');
