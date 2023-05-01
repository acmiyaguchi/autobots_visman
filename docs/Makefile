all: report-spr23

slides-0: slides_spr23_00_docker.md
	pandoc -s \
		-t beamer \
		-o rendered/slides_spr23_00_docker.pdf \
		slides_spr23_00_docker.md

slides-1:
	pandoc -s \
		-t beamer \
		-o rendered/slides_spr23_01_handy_sim.pdf \
		slides_spr23_01_handy_sim.md

slides-2:
	pandoc -s \
		-t beamer \
		-o rendered/slides_spr23_02_pick_and_place.pdf \
		slides_spr23_02_pick_and_place.md

report-spr23: report_spr23.md
	pandoc -s -o rendered/tmp.pdf report_spr23.md
	pdftk \
		rendered/tmp.pdf \
		rendered/slides_spr23_00_docker.pdf \
		rendered/slides_spr23_01_handy_sim.pdf \
		rendered/slides_spr23_02_pick_and_place.pdf \
		cat output rendered/report_spr23.pdf
	rm rendered/tmp.pdf
