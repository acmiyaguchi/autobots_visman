.PHONY: all clean slides diagrams

slides:
	pandoc -s \
		-t beamer \
		-F mermaid-filter \
		-o rendered/slides_spr23_00_docker.pdf \
		slides_spr23_00_docker.md