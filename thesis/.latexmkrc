$pdf_mode = 1;

# If latexmk is invoked with no file argument, build only main.tex.
@default_files = ('main.tex');

$out_dir = 'build';
$aux_dir = 'build';

# Build with pdfLaTeX for BasicTeX robustness.
$pdflatex = 'pdflatex -synctex=1 -interaction=nonstopmode -file-line-error %O %S';

# Use biber with biblatex.
$bibtex_use = 2;

# Keep logs in build/ too.
$clean_full_ext = 'acn acr alg aux bbl bcf blg fdb_latexmk fls glg glo gls idx ilg ind ist lof log lot nav out run.xml snm synctex.gz toc';
