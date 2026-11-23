| Hold | Capture | Method | Mean dist. mm (acc / compl.) | 95% mm | F@2mm | Size error mm (x/y/z) | Volume % | Time s | Watertight |
|---|---|---|---|---|---|---|---|---|---|
| sloper | σ 1.5 mm, 48 frames | TSDF 2 mm (1.3.0) | 0.65 / 0.77 | 2.0 | 0.939 | -4.1 / -2.0 / 0.1 | -4.5 | 0.41 | yes |
| sloper | σ 1.5 mm, 48 frames | TSDF mesh as PLY + table → PLY path | 0.62 / 0.72 | 1.8 | 0.950 | -4.6 / -3.0 / 0.1 | -5.6 | 1.23 | yes |
| sloper | σ 1.5 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | failed: A síkból nem emelkedik ki fogás a felvétel közepén. Tartsd középen a fogást, és járd körbe. | | | | | 0.49 | |
| sloper | σ 4 mm, 48 frames | TSDF 2 mm (1.3.0) | 1.10 / 1.07 | 3.5 | 0.807 | -9.0 / -6.5 / 0.3 | -8.5 | 0.32 | yes |
| sloper | σ 4 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | 4.89 / 4.90 | 10.0 | 0.319 | -26.1 / -24.7 / 18.4 | -9.0 | 0.61 | yes |
| sloper | σ 1.5 mm, 12 frames | TSDF 2 mm (1.3.0) | 0.53 / 0.62 | 1.6 | 0.961 | -3.0 / -2.2 / 0.2 | -4.6 | 0.15 | yes |
| sloper | σ 1.5 mm, 12 frames | Roomantic occupancy 1 cm → PLY path | failed: A PLY-fájlban nincs a síkból kiemelkedő tárgy. | | | | | 0.15 | |
| sloper | σ 1.5 mm, drift 1 mm | TSDF 2 mm (1.3.0) | 0.54 / 0.63 | 1.7 | 0.963 | -4.3 / -1.4 / 0.2 | -4.0 | 0.30 | yes |
| sloper | σ 1.5 mm, drift 1 mm | Roomantic occupancy 1 cm → PLY path | failed: A síkból nem emelkedik ki fogás a felvétel közepén. Tartsd középen a fogást, és járd körbe. | | | | | 0.18 | |
| sloper | one top-down photo, perfect depth, true size given | Single-photo relief (1.2.0), oracle | 1.14 / 1.30 | 3.5 | 0.741 | 0.0 / 0.5 / 0.0 | 12.2 | 0.25 | – |
| jug | σ 1.5 mm, 48 frames | TSDF 2 mm (1.3.0) | 0.65 / 0.75 | 2.0 | 0.938 | -0.7 / -1.3 / -0.5 | -3.1 | 0.34 | yes |
| jug | σ 1.5 mm, 48 frames | TSDF mesh as PLY + table → PLY path | 0.71 / 0.80 | 2.2 | 0.923 | -1.8 / -2.7 / -0.7 | -4.2 | 0.71 | yes |
| jug | σ 1.5 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | 8.09 / 7.57 | 10.0 | 0.118 | 157.3 / 22.2 / -8.3 | -13.7 | 0.45 | yes |
| jug | σ 4 mm, 48 frames | TSDF 2 mm (1.3.0) | 0.97 / 1.03 | 3.1 | 0.830 | -5.2 / -5.0 / -0.5 | -5.5 | 0.30 | yes |
| jug | σ 4 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | 7.27 / 9.41 | 10.0 | 0.020 | -50.8 / -45.0 / -31.5 | -93.3 | 0.67 | yes |
| jug | σ 1.5 mm, 12 frames | TSDF 2 mm (1.3.0) | 0.68 / 0.76 | 2.1 | 0.935 | 0.2 / -2.0 / -0.4 | -3.3 | 0.15 | yes |
| jug | σ 1.5 mm, 12 frames | Roomantic occupancy 1 cm → PLY path | failed: A PLY-fájlban nincs a síkból kiemelkedő tárgy. | | | | | 0.20 | |
| jug | σ 1.5 mm, drift 1 mm | TSDF 2 mm (1.3.0) | 0.74 / 0.87 | 2.4 | 0.901 | -1.4 / -1.2 / -0.6 | -4.0 | 0.29 | yes |
| jug | σ 1.5 mm, drift 1 mm | Roomantic occupancy 1 cm → PLY path | 6.32 / 9.87 | 9.3 | 0.000 | -91.7 / -67.3 / -41.2 | -99.5 | 0.47 | yes |
| jug | one top-down photo, perfect depth, true size given | Single-photo relief (1.2.0), oracle | 2.50 / 2.82 | 8.4 | 0.553 | -0.0 / -4.5 / 0.0 | 29.1 | 0.33 | – |
| crimp | σ 1.5 mm, 48 frames | TSDF 2 mm (1.3.0) | 0.51 / 0.63 | 1.5 | 0.977 | 1.1 / -0.6 / 0.1 | -6.7 | 0.15 | yes |
| crimp | σ 1.5 mm, 48 frames | TSDF mesh as PLY + table → PLY path | 0.51 / 0.62 | 1.4 | 0.980 | 0.1 / -1.2 / -0.3 | -7.5 | 0.30 | yes |
| crimp | σ 1.5 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | failed: A PLY-fájlban nincs a síkból kiemelkedő tárgy. | | | | | 0.12 | |
| crimp | σ 4 mm, 48 frames | TSDF 2 mm (1.3.0) | 0.80 / 0.83 | 2.2 | 0.916 | 2.8 / -1.2 / 0.3 | -5.9 | 0.16 | yes |
| crimp | σ 4 mm, 48 frames | Roomantic occupancy 1 cm → PLY path | 9.12 / 5.66 | 10.0 | 0.080 | 428.4 / -7.8 / 7.4 | 308.2 | 0.39 | yes |
| crimp | σ 1.5 mm, 12 frames | TSDF 2 mm (1.3.0) | 0.53 / 0.64 | 1.5 | 0.977 | 1.3 / 0.0 / 0.1 | -7.0 | 0.08 | yes |
| crimp | σ 1.5 mm, 12 frames | Roomantic occupancy 1 cm → PLY path | failed: A PLY-fájlban nincs a síkból kiemelkedő tárgy. | | | | | 0.08 | |
| crimp | σ 1.5 mm, drift 1 mm | TSDF 2 mm (1.3.0) | 0.66 / 0.80 | 2.4 | 0.908 | 0.8 / 0.4 / -0.3 | -7.6 | 0.14 | yes |
| crimp | σ 1.5 mm, drift 1 mm | Roomantic occupancy 1 cm → PLY path | failed: A PLY-fájlban nincs a síkból kiemelkedő tárgy. | | | | | 0.12 | |
| crimp | one top-down photo, perfect depth, true size given | Single-photo relief (1.2.0), oracle | 3.70 / 0.97 | 8.8 | 0.493 | -0.6 / -3.3 / 0.0 | -74.9 | 0.29 | – |
