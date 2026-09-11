# Q1 正文的版式与证据约定

Reference: D:/2026 MCM1/2026-MCM1/B/08_paper_submission/国赛2026全文论文模板.docx
SHA256: f4c3066fe1dfd4b6045f4389d010ac7c0a17933ef15918743cb2e80e34cd8f0a
Reference has one section. Existing Word-exported preview and all seven retained PNGs inspected; pages 6 and 7 are duplicate appendix patterns in saved render folder. Use body chapter pattern from page 3, table pattern from pages 2 and 3, figure pattern from page 4. This deliverable is only Q1, so abstract, other questions, full-paper declarations/references and appendices are intentionally omitted.

Page: A4 portrait 11906 by 16838 twips; four margins 1417 twips; header 567, footer 709 twips. Preserve sectPr, footer PAGE field and starting page 1 for the standalone excerpt. No decorative header. After insertion into full paper, page field follows full-paper pagination.
Typography: preserve all styles byte-for-byte. Normal Songti 12pt, Times New Roman Latin, 1.25 lines, 24pt first indent, justified, zero before/after. Heading 1 Heiti 14pt, spacing14/7; Heading2 Heiti12pt9/4; Heading3 Heiti12pt6/3. All black. Equation native OMML, centered on 8cm tab, number on 16cm right tab. Caption10.5pt centered. Table Header and Table Text10.5pt, gray borders D9D9D9, header F2F2F2, padding80/100twips, repeated header, no split rows.

Slots: replace word/document.xml body children with Q1 chapter, keeping original final sectPr. First paragraph uses Heading1 as a paper section; no extra cover/title page. Native mathematical expressions can be added in Equation paragraphs, equations numbered5-1 onward. Proofs use connected Normal paragraphs. Table and figure may clone template patterns. Remove all placeholder paragraphs and example diagrams from the visible body. Add new figure relationship/media; preserve existing unrelated parts byte-for-byte. Four logical pages planned: region model, geometry proofs, algorithm and verification, synthetic case results.

Package: preserve every original part except word/document.xml, word/_rels/document.xml.rels and docProps/core.xml. New image part permitted. Retain original styles, theme, settings, numbering, footer, customXml, old media, thumbnail and content-types. Core metadata is editable for anonymity and correct subject. Builder saves original package entries and substitutes only these parts; emits inventory verification JSON. All authoring tools and evidence files are internal, not final deliverables.

Evidence: q1_paper_tools/RESULT_REGISTRY.json and CLAIM_EVIDENCE.csv. Synthetic case inputs and geometric outputs recalculated; random-sample statistics checked against saved records; 18 unit tests rerun. Mathematical results are justified in the body. Report source comparison differences as numerical consistency, never source-location error. Figures sourced from saved validated PNG without modifying image content.

Render: Word native PDF export through a hidden dedicated instance; packaged render_docx.py rasterizes that exact PDF through a conversion adapter, consistent with existing template workflow. Inspect all final pages, equations, tables and figure, and check package preservation and no placeholders.
