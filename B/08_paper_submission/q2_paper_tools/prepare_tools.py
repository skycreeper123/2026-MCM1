from pathlib import Path
HERE=Path(__file__).parent
prev=HERE.parent/'q1_paper_tools'
s=(prev/'build_q1.py').read_text(encoding='utf-8')
imports=s[:s.index('HERE=')]
helpers=s[s.index('def para('):s.index("h('5 问题一")].replace('（5-{n}）','（6-{n}）')
(HERE/'doc_helpers.py').write_text(imports+'doc=None\n'+helpers,encoding='utf-8')
for old,new in [('export_q1.ps1','export_q2.ps1'),('render_q1.py','render_q2.py')]:
    (HERE/new).write_text((prev/old).read_text(encoding='utf-8').replace('Q1','Q2').replace('q1','q2'),encoding='utf-8')
contract=(prev/'artifact.md').read_text(encoding='utf-8').replace('Q1','Q2').replace('q1_paper_tools','q2_paper_tools').replace('5-1','6-1')
contract+='''

Q2-specific slot contract: retain the same verified reference and styles. Replace only the visible body with chapter 6. Preserve Q1正文.docx byte-for-byte. Planned five pages: initial outer polygon and fixed reception guarantee; minimax radius and continuous response bounds; multistart solving and first-reception-information extension; typical candidate maps and coordinate/metric table; validation comparison, reliability and limitations. If needed use a sixth page for readable validation. Native equations numbered 6-1 onward. Use two original validated PNGs, q2_v3_01_good_regions and q2_v3_02_paired_improvement. Do not append full-paper AI declarations or reference/appendix placeholders to this excerpt. All experimental numerical claims are read from the scoped RESULT_REGISTRY and tied to CLAIM_EVIDENCE. Distinguish 200 random paired statistics from safety checks across all 220 cases. No claim of global optimality or hard real-time guarantees.
'''
(HERE/'artifact.md').write_text(contract,encoding='utf-8')
print('Prepared Q2 style helpers and rendering adapters.')
