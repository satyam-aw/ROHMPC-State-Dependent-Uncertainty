# State-dependent ROHMPC manuscript

The manuscript follows the section organization of Kohler, Muller, and Allgower (2021): Introduction; Preliminaries; Estimation Error Bounds; Robust Output-Feedback MPC; Numerical Example; Conclusion.

Files required: state_dependent_rohmpc.tex and falcon_tube_comparison.png. Upload both to Overleaf, or run from this directory:

    latexmk -pdf state_dependent_rohmpc.tex

References are embedded; no .bib file is required. The article uses Roman-numbered sections and lettered subsections in a readable single-column layout, rather than reproducing the reference paper's two-column typesetting.

Status: research draft. Theorems are conditional. Falcon results are saved radius-ODE experiments, not closed-loop MPC simulations. No new SDP or plant simulation was run to prepare this manuscript.
