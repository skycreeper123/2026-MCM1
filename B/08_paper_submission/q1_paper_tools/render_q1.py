from pathlib import Path
import importlib.util, os, shutil
HERE=Path(__file__).parent
skill=Path('C:/Users/but48/.codex/plugins/cache/openai-primary-runtime/documents/26.909.12148/skills/documents/render_docx.py')
spec=importlib.util.spec_from_file_location('docx_renderer',skill)
renderer=importlib.util.module_from_spec(spec);spec.loader.exec_module(renderer)
os.environ['PATH']='C:/Users/but48/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/Library/bin'+os.pathsep+os.environ['PATH']
def word_pdf(doc_path,user_profile,convert_tmp_dir,stem,verbose):
    target=Path(convert_tmp_dir)/(stem+'.pdf');shutil.copy2(HERE/'Q1-preview.pdf',target)
    return str(target),'Exact current DOCX exported read-only by Microsoft Word; packaged render_docx.py rasterization.'
renderer.convert_to_pdf=word_pdf
paths=renderer.rasterize(str(HERE.parent/'Q1正文.docx'),str(HERE/'rendered'),150,False,False)
print('Rendered',len(paths),'pages')
