# SEARCH_LOG — systematic review query ledger (§12.0)

Every query is logged with source, date, and hits worth a librarian note.
Coverage areas from §12.0; the matrix below tracks which area × source
combinations have been swept. **Status: seeded 2026-07-17, no queries executed
yet — librarian sweeps begin alongside P1.**

## Coverage matrix (area × source)

Sources: GS = Google Scholar, IX = IEEE Xplore, AX = arXiv, AAS/AIAA = conf.
proceedings, NTRS = NASA technical reports.

| # | Coverage area | GS | IX | AX | AAS/AIAA | NTRS |
|---|---|---|---|---|---|---|
| 1 | limb/horizon-based optical navigation | — | — | — | — | — |
| 2 | sub-pixel edge & limb localization | — | — | — | — | — |
| 3 | contour detection on planetary/space imagery (classical & learned) | — | — | — | — | — |
| 4 | CNN/learned OPNAV & pose estimation | — | — | — | — | — |
| 5 | disk/diameter/size estimation from imagery | — | — | — | — | — |
| 6 | flown autonomous onboard systems (DS1 AutoNav, OSIRIS-REx, DART) + compute constraints | — | — | — | — | — |
| 7 | detector+geometry hybrid pipelines | — | — | — | — | — |

## Planned seed queries (draft; refine per area on first sweep)

- "horizon-based optical navigation" limb spacecraft
- "sub-pixel" limb localization planetary edge Zernike
- Cassini ISS contour detection segmentation
- planetary disk center estimation image navigation CNN
- apparent diameter measurement planetary image sub-pixel
- "AutoNav" Deep Space 1 onboard optical navigation compute
- OSIRIS-REx OpNav limb centroid; DART SMART Nav
- bounding box refinement geometric limb hybrid detection spacecraft

## Query log (append-only)

| date | source | query | hits kept | notes file(s) |
|---|---|---|---|---|

## Seed bibliography status (§12.1)

| key | citation | status |
|---|---|---|
| christian2021 | Christian, IEEE Access 2021, horizon-based OPNAV tutorial | to verify |
| christian2017 | Christian, JSR 2017, accurate planetary limb localization | to verify |
| ghosal1993 | Ghosal & Mehrotra, PR 1993, Zernike subpixel edges | to verify |
| fitzgibbon1999 | Fitzgibbon/Pilu/Fisher, TPAMI 1999, direct LS ellipses | to verify |
| owen2011 | Owen, AAS 11-215, methods of optical navigation | to verify |
| owen2003 | Owen 2003 Cassini ISS geometric calibration + West et al. 2010 CISSCAL | to verify |
| porco2004 | Porco et al. 2004, SSR — already cited by instruments.csv | verified (submodule) |
| archinal2018 | Archinal et al. 2018, IAU WGCCRE | to verify |
| bhaskaran | Bhaskaran et al., DS1 AutoNav | to verify |
| arxiv1908.08279 | H-ELM + DenseCRF Cassini contours — first baseline (§12.3) | to verify + extract numbers → baseline_1908.08279.md |
