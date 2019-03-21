CMD = xelatex
DOC = hw2
all: 
	$(CMD) $(DOC) && bibtex $(DOC) && $(CMD) $(DOC) && $(CMD) $(DOC) 

clean:
	rm *.aux *.log *.blg *.bbl *.ps *.log *.dvi
