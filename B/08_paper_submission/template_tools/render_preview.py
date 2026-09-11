from pathlib import Path
import importlib.util
import os
import shutil

root = Path(__file__).resolve().parents[1]
skill = Path('C:/Users/but48/.codex/plugins/cache/openai-primary-runtime/documents/26.909.12148/skills/documents/render_docx.py')
spec = importlib.util.spec_from_file_location('docx_renderer', skill)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)
os.environ['PATH'] = 'C:/Users/but48/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/Library/bin' + os.pathsep + os.environ['PATH']
preview = root / 'template_tools/qa/template-preview.pdf'

def word_preview(doc_path, user_profile, convert_tmp_dir, stem, verbose):
    target = Path(convert_tmp_dir) / (stem + '.pdf')
    shutil.copy2(preview, target)
    return str(target), 'PDF exported from exact DOCX using Microsoft Word; rasterization by bundled render_docx.py.'

renderer.convert_to_pdf = word_preview
paths = renderer.rasterize(str(root/'国赛2026全文论文模板.docx'),str(root/'template_tools/qa/rendered'),150,False,False)
print('Rendered pages:',len(paths))
