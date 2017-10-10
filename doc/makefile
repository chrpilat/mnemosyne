ROOT=howto

all: doc

create-pdf: $(ROOT).tex
	pdflatex --shell-escape  $(ROOT)
	pdflatex --shell-escape  $(ROOT)

remove-extra-files:
	@rm -rf $(ROOT).synctex.gz $(ROOT).listing _minted-howto $(ROOT).sta
	@rubber --clean $(ROOT)

doc: create-pdf remove-extra-files

clean: remove-extra-files
	@rm -f $(ROOT).pdf *.bbl *.blg
