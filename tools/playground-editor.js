/* playground-editor.js — CodeMirror 6 (pinned jsdelivr +esm, deduplicated by the import
 * map in playground.html) with a plain <textarea> fallback if the CDN modules fail.
 *
 * API: createEditor(host, {doc, onRun, onChange}) -> {getValue, setValue, focus, hasFocus, kind}
 */

const CM = {
  codemirror: 'https://cdn.jsdelivr.net/npm/codemirror@6.0.2/+esm',
  view: 'https://cdn.jsdelivr.net/npm/@codemirror/view@6.37.2/+esm',
  state: 'https://cdn.jsdelivr.net/npm/@codemirror/state@6.5.2/+esm',
  commands: 'https://cdn.jsdelivr.net/npm/@codemirror/commands@6.8.1/+esm',
  python: 'https://cdn.jsdelivr.net/npm/@codemirror/lang-python@6.2.1/+esm',
  oneDark: 'https://cdn.jsdelivr.net/npm/@codemirror/theme-one-dark@6.1.3/+esm',
};

function textareaEditor(host, { doc, onRun, onChange }) {
  const ta = document.createElement('textarea');
  ta.className = 'pg-textarea';
  ta.spellcheck = false;
  ta.value = doc;
  ta.setAttribute('aria-label', 'Python 코드');
  ta.addEventListener('keydown', (e) => {
    if (e.key === 'Tab' && !e.shiftKey) {
      e.preventDefault();
      const s = ta.selectionStart;
      ta.setRangeText('    ', s, ta.selectionEnd, 'end');
      onChange && onChange(ta.value);
    } else if (e.key === 'Enter' && (e.ctrlKey || e.metaKey || e.shiftKey)) {
      e.preventDefault();
      onRun && onRun();
    } else if (e.key === 'Escape') {
      ta.blur();
    }
  });
  ta.addEventListener('input', () => onChange && onChange(ta.value));
  host.appendChild(ta);
  return {
    kind: 'textarea',
    getValue: () => ta.value,
    setValue: (v) => { ta.value = v; onChange && onChange(v); },
    focus: () => ta.focus(),
    hasFocus: () => document.activeElement === ta,
  };
}

export async function createEditor(host, opts) {
  try {
    const [cm, viewMod, stateMod, commands, py, dark] = await Promise.all([
      import(CM.codemirror), import(CM.view), import(CM.state), import(CM.commands), import(CM.python), import(CM.oneDark),
    ]);
    const { EditorView, basicSetup } = cm;
    const { keymap } = viewMod;
    const { Prec, EditorState } = stateMod;
    const runKeys = keymap.of([
      { key: 'Mod-Enter', run: () => { opts.onRun && opts.onRun(); return true; } },
      { key: 'Shift-Enter', run: () => { opts.onRun && opts.onRun(); return true; } },
    ]);
    // Esc leaves the editor (after autocomplete etc. had their chance) so Space = E-stop works
    const escBlur = Prec.lowest(keymap.of([{ key: 'Escape', run: (v) => { v.contentDOM.blur(); return true; } }]));
    const view = new EditorView({
      parent: host,
      state: EditorState.create({
        doc: opts.doc,
        extensions: [
          Prec.highest(runKeys),
          basicSetup,
          keymap.of([commands.indentWithTab]),
          escBlur,
          py.python(),
          dark.oneDark,
          EditorState.tabSize.of(4),
          EditorView.updateListener.of((u) => {
            if (u.docChanged && opts.onChange) opts.onChange(u.state.doc.toString());
          }),
          EditorView.contentAttributes.of({ 'aria-label': 'Python 코드' }),
        ],
      }),
    });
    return {
      kind: 'codemirror',
      getValue: () => view.state.doc.toString(),
      setValue: (v) => view.dispatch({ changes: { from: 0, to: view.state.doc.length, insert: v } }),
      focus: () => view.focus(),
      hasFocus: () => view.hasFocus,
    };
  } catch (e) {
    console.warn('[playground] CodeMirror failed to load, using textarea fallback:', e);
    host.textContent = '';
    const ed = textareaEditor(host, opts);
    ed.loadError = String(e && e.message ? e.message : e);
    return ed;
  }
}
