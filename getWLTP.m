function pf = getWLTP()
    t = [0
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
75
76
77
78
79
80
81
82
83
84
85
86
87
88
89
90
91
92
93
94
95
96
97
98
99
100
101
102
103
104
105
106
107
108
109
110
111
112
113
114
115
116
117
118
119
120
121
122
123
124
125
126
127
128
129
130
131
132
133
134
135
136
137
138
139
140
141
142
143
144
145
146
147
148
149
150
151
152
153
154
155
156
157
158
159
160
161
162
163
164
165
166
167
168
169
170
171
172
173
174
175
176
177
178
179
180
181
182
183
184
185
186
187
188
189
190
191
192
193
194
195
196
197
198
199
200
201
202
203
204
205
206
207
208
209
210
211
212
213
214
215
216
217
218
219
220
221
222
223
224
225
226
227
228
229
230
231
232
233
234
235
236
237
238
239
240
241
242
243
244
245
246
247
248
249
250
251
252
253
254
255
256
257
258
259
260
261
262
263
264
265
266
267
268
269
270
271
272
273
274
275
276
277
278
279
280
281
282
283
284
285
286
287
288
289
290
291
292
293
294
295
296
297
298
299
300
301
302
303
304
305
306
307
308
309
310
311
312
313
314
315
316
317
318
319
320
321
322
323
324
325
326
327
328
329
330
331
332
333
334
335
336
337
338
339
340
341
342
343
344
345
346
347
348
349
350
351
352
353
354
355
356
357
358
359
360
361
362
363
364
365
366
367
368
369
370
371
372
373
374
375
376
377
378
379
380
381
382
383
384
385
386
387
388
389
390
391
392
393
394
395
396
397
398
399
400
401
402
403
404
405
406
407
408
409
410
411
412
413
414
415
416
417
418
419
420
421
422
423
424
425
426
427
428
429
430
431
432
433
434
435
436
437
438
439
440
441
442
443
444
445
446
447
448
449
450
451
452
453
454
455
456
457
458
459
460
461
462
463
464
465
466
467
468
469
470
471
472
473
474
475
476
477
478
479
480
481
482
483
484
485
486
487
488
489
490
491
492
493
494
495
496
497
498
499
500
501
502
503
504
505
506
507
508
509
510
511
512
513
514
515
516
517
518
519
520
521
522
523
524
525
526
527
528
529
530
531
532
533
534
535
536
537
538
539
540
541
542
543
544
545
546
547
548
549
550
551
552
553
554
555
556
557
558
559
560
561
562
563
564
565
566
567
568
569
570
571
572
573
574
575
576
577
578
579
580
581
582
583
584
585
586
587
588
589
590
591
592
593
594
595
596
597
598
599
600
601
602
603
604
605
606
607
608
609
610
611
612
613
614
615
616
617
618
619
620
621
622
623
624
625
626
627
628
629
630
631
632
633
634
635
636
637
638
639
640
641
642
643
644
645
646
647
648
649
650
651
652
653
654
655
656
657
658
659
660
661
662
663
664
665
666
667
668
669
670
671
672
673
674
675
676
677
678
679
680
681
682
683
684
685
686
687
688
689
690
691
692
693
694
695
696
697
698
699
700
701
702
703
704
705
706
707
708
709
710
711
712
713
714
715
716
717
718
719
720
721
722
723
724
725
726
727
728
729
730
731
732
733
734
735
736
737
738
739
740
741
742
743
744
745
746
747
748
749
750
751
752
753
754
755
756
757
758
759
760
761
762
763
764
765
766
767
768
769
770
771
772
773
774
775
776
777
778
779
780
781
782
783
784
785
786
787
788
789
790
791
792
793
794
795
796
797
798
799
800
801
802
803
804
805
806
807
808
809
810
811
812
813
814
815
816
817
818
819
820
821
822
823
824
825
826
827
828
829
830
831
832
833
834
835
836
837
838
839
840
841
842
843
844
845
846
847
848
849
850
851
852
853
854
855
856
857
858
859
860
861
862
863
864
865
866
867
868
869
870
871
872
873
874
875
876
877
878
879
880
881
882
883
884
885
886
887
888
889
890
891
892
893
894
895
896
897
898
899
900
901
902
903
904
905
906
907
908
909
910
911
912
913
914
915
916
917
918
919
920
921
922
923
924
925
926
927
928
929
930
931
932
933
934
935
936
937
938
939
940
941
942
943
944
945
946
947
948
949
950
951
952
953
954
955
956
957
958
959
960
961
962
963
964
965
966
967
968
969
970
971
972
973
974
975
976
977
978
979
980
981
982
983
984
985
986
987
988
989
990
991
992
993
994
995
996
997
998
999
1000
1001
1002
1003
1004
1005
1006
1007
1008
1009
1010
1011
1012
1013
1014
1015
1016
1017
1018
1019
1020
1021
1022
1023
1024
1025
1026
1027
1028
1029
1030
1031
1032
1033
1034
1035
1036
1037
1038
1039
1040
1041
1042
1043
1044
1045
1046
1047
1048
1049
1050
1051
1052
1053
1054
1055
1056
1057
1058
1059
1060
1061
1062
1063
1064
1065
1066
1067
1068
1069
1070
1071
1072
1073
1074
1075
1076
1077
1078
1079
1080
1081
1082
1083
1084
1085
1086
1087
1088
1089
1090
1091
1092
1093
1094
1095
1096
1097
1098
1099
1100
1101
1102
1103
1104
1105
1106
1107
1108
1109
1110
1111
1112
1113
1114
1115
1116
1117
1118
1119
1120
1121
1122
1123
1124
1125
1126
1127
1128
1129
1130
1131
1132
1133
1134
1135
1136
1137
1138
1139
1140
1141
1142
1143
1144
1145
1146
1147
1148
1149
1150
1151
1152
1153
1154
1155
1156
1157
1158
1159
1160
1161
1162
1163
1164
1165
1166
1167
1168
1169
1170
1171
1172
1173
1174
1175
1176
1177
1178
1179
1180
1181
1182
1183
1184
1185
1186
1187
1188
1189
1190
1191
1192
1193
1194
1195
1196
1197
1198
1199
1200
1201
1202
1203
1204
1205
1206
1207
1208
1209
1210
1211
1212
1213
1214
1215
1216
1217
1218
1219
1220
1221
1222
1223
1224
1225
1226
1227
1228
1229
1230
1231
1232
1233
1234
1235
1236
1237
1238
1239
1240
1241
1242
1243
1244
1245
1246
1247
1248
1249
1250
1251
1252
1253
1254
1255
1256
1257
1258
1259
1260
1261
1262
1263
1264
1265
1266
1267
1268
1269
1270
1271
1272
1273
1274
1275
1276
1277
1278
1279
1280
1281
1282
1283
1284
1285
1286
1287
1288
1289
1290
1291
1292
1293
1294
1295
1296
1297
1298
1299
1300
1301
1302
1303
1304
1305
1306
1307
1308
1309
1310
1311
1312
1313
1314
1315
1316
1317
1318
1319
1320
1321
1322
1323
1324
1325
1326
1327
1328
1329
1330
1331
1332
1333
1334
1335
1336
1337
1338
1339
1340
1341
1342
1343
1344
1345
1346
1347
1348
1349
1350
1351
1352
1353
1354
1355
1356
1357
1358
1359
1360
1361
1362
1363
1364
1365
1366
1367
1368
1369
1370
1371
1372
1373
1374
1375
1376
1377
1378
1379
1380
1381
1382
1383
1384
1385
1386
1387
1388
1389
1390
1391
1392
1393
1394
1395
1396
1397
1398
1399
1400
1401
1402
1403
1404
1405
1406
1407
1408
1409
1410
1411
1412
1413
1414
1415
1416
1417
1418
1419
1420
1421
1422
1423
1424
1425
1426
1427
1428
1429
1430
1431
1432
1433
1434
1435
1436
1437
1438
1439
1440
1441
1442
1443
1444
1445
1446
1447
1448
1449
1450
1451
1452
1453
1454
1455
1456
1457
1458
1459
1460
1461
1462
1463
1464
1465
1466
1467
1468
1469
1470
1471
1472
1473
1474
1475
1476
1477
1478
1479
1480
1481
1482
1483
1484
1485
1486
1487
1488
1489
1490
1491
1492
1493
1494
1495
1496
1497
1498
1499
1500
1501
1502
1503
1504
1505
1506
1507
1508
1509
1510
1511
1512
1513
1514
1515
1516
1517
1518
1519
1520
1521
1522
1523
1524
1525
1526
1527
1528
1529
1530
1531
1532
1533
1534
1535
1536
1537
1538
1539
1540
1541
1542
1543
1544
1545
1546
1547
1548
1549
1550
1551
1552
1553
1554
1555
1556
1557
1558
1559
1560
1561
1562
1563
1564
1565
1566
1567
1568
1569
1570
1571
1572
1573
1574
1575
1576
1577
1578
1579
1580
1581
1582
1583
1584
1585
1586
1587
1588
1589
1590
1591
1592
1593
1594
1595
1596
1597
1598
1599
1600
1601
1602
1603
1604
1605
1606
1607
1608
1609
1610
1611
1612
1613
1614
1615
1616
1617
1618
1619
1620
1621
1622
1623
1624
1625
1626
1627
1628
1629
1630
1631
1632
1633
1634
1635
1636
1637
1638
1639
1640
1641
1642
1643
1644
1645
1646
1647
1648
1649
1650
1651
1652
1653
1654
1655
1656
1657
1658
1659
1660
1661
1662
1663
1664
1665
1666
1667
1668
1669
1670
1671
1672
1673
1674
1675
1676
1677
1678
1679
1680
1681
1682
1683
1684
1685
1686
1687
1688
1689
1690
1691
1692
1693
1694
1695
1696
1697
1698
1699
1700
1701
1702
1703
1704
1705
1706
1707
1708
1709
1710
1711
1712
1713
1714
1715
1716
1717
1718
1719
1720
1721
1722
1723
1724
1725
1726
1727
1728
1729
1730
1731
1732
1733
1734
1735
1736
1737
1738
1739
1740
1741
1742
1743
1744
1745
1746
1747
1748
1749
1750
1751
1752
1753
1754
1755
1756
1757
1758
1759
1760
1761
1762
1763
1764
1765
1766
1767
1768
1769
1770
1771
1772
1773
1774
1775
1776
1777
1778
1779
1780
1781
1782
1783
1784
1785
1786
1787
1788
1789
1790
1791
1792
1793
1794
1795
1796
1797
1798
1799
1800
];




v = [0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.2 
1.7 
5.4 
9.9 
13.1 
16.9 
21.7 
26.0 
27.5 
28.1 
28.3 
28.8 
29.1 
30.8 
31.9 
34.1 
36.6 
39.1 
41.3 
42.5 
43.3 
43.9 
44.4 
44.5 
44.2 
42.7 
39.9 
37.0 
34.6 
32.3 
29.0 
25.1 
22.2 
20.9 
20.4 
19.5 
18.4 
17.8 
17.8 
17.4 
15.7 
13.1 
12.1 
12.0 
12.0 
12.0 
12.3 
12.6 
14.7 
15.3 
15.9 
16.2 
17.1 
17.8 
18.1 
18.4 
20.3 
23.2 
26.5 
29.8 
32.6 
34.4 
35.5 
36.4 
37.4 
38.5 
39.3 
39.5 
39.0 
38.5 
37.3 
37.0 
36.7 
35.9 
35.3 
34.6 
34.2 
31.9 
27.3 
22.0 
17.0 
14.2 
12.0 
9.1 
5.8 
3.6 
2.2 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.2 
1.9 
6.1 
11.7 
16.4 
18.9 
19.9 
20.8 
22.8 
25.4 
27.7 
29.2 
29.8 
29.4 
27.2 
22.6 
17.3 
13.3 
12.0 
12.6 
14.1 
17.2 
20.1 
23.4 
25.5 
27.6 
29.5 
31.1 
32.1 
33.2 
35.2 
37.2 
38.0 
37.4 
35.1 
31.0 
27.1 
25.3 
25.1 
25.9 
27.8 
29.2 
29.6 
29.5 
29.2 
28.3 
26.1 
23.6 
21.0 
18.9 
17.1 
15.7 
14.5 
13.7 
12.9 
12.5 
12.2 
12.0 
12.0 
12.0 
12.0 
12.5 
13.0 
14.0 
15.0 
16.5 
19.0 
21.2 
23.8 
26.9 
29.6 
32.0 
35.2 
37.5 
39.2 
40.5 
41.6 
43.1 
45.0 
47.1 
49.0 
50.6 
51.8 
52.7 
53.1 
53.5 
53.8 
54.2 
54.8 
55.3 
55.8 
56.2 
56.5 
56.5 
56.2 
54.9 
52.9 
51.0 
49.8 
49.2 
48.4 
46.9 
44.3 
41.5 
39.5 
37.0 
34.6 
32.3 
29.0 
25.1 
22.2 
20.9 
20.4 
19.5 
18.4 
17.8 
17.8 
17.4 
15.7 
14.5 
15.4 
17.9 
20.6 
23.2 
25.7 
28.7 
32.5 
36.1 
39.0 
40.8 
42.9 
44.4 
45.9 
46.0 
45.6 
45.3 
43.7 
40.8 
38.0 
34.4 
30.9 
25.5 
21.4 
20.2 
22.9 
26.6 
30.2 
34.1 
37.4 
40.7 
44.0 
47.3 
49.2 
49.8 
49.2 
48.1 
47.3 
46.8 
46.7 
46.8 
47.1 
47.3 
47.3 
47.1 
46.6 
45.8 
44.8 
43.3 
41.8 
40.8 
40.3 
40.1 
39.7 
39.2 
38.5 
37.4 
36.0 
34.4 
33.0 
31.7 
30.0 
28.0 
26.1 
25.6 
24.9 
24.9 
24.3 
23.9 
23.9 
23.6 
23.3 
20.5 
17.5 
16.9 
16.7 
15.9 
15.6 
15.0 
14.5 
14.3 
14.5 
15.4 
17.8 
21.1 
24.1 
25.0 
25.3 
25.5 
26.4 
26.6 
27.1 
27.7 
28.1 
28.2 
28.1 
28.0 
27.9 
27.9 
28.1 
28.2 
28.0 
26.9 
25.0 
23.2 
21.9 
21.1 
20.7 
20.7 
20.8 
21.2 
22.1 
23.5 
24.3 
24.5 
23.8 
21.3 
17.7 
14.4 
11.9 
10.2 
8.9 
8.0 
7.2 
6.1 
4.9 
3.7 
2.3 
0.9 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.5 
2.1 
4.8 
8.3 
12.3 
16.6 
20.9 
24.2 
25.6 
25.6 
24.9 
23.3 
21.6 
20.2 
18.7 
17.0 
15.3 
14.2 
13.9 
14.0 
14.2 
14.5 
14.9 
15.9 
17.4 
18.7 
19.1 
18.8 
17.6 
16.6 
16.2 
16.4 
17.2 
19.1 
22.6 
27.4 
31.6 
33.4 
33.5 
32.8 
31.9 
31.3 
31.1 
30.6 
29.2 
26.7 
23.0 
18.2 
12.9 
7.7 
3.8 
1.3 
0.2 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.5 
2.5 
6.6 
11.8 
16.8 
20.5 
21.9 
21.9 
21.3 
20.3 
19.2 
17.8 
15.5 
11.9 
7.6 
4.0 
2.0 
1.0 
0.0 
0.0 
0.0 
0.2 
1.2 
3.2 
5.2 
8.2 
13.0 
18.8 
23.1 
24.5 
24.5 
24.3 
23.6 
22.3 
20.1 
18.5 
17.2 
16.3 
15.4 
14.7 
14.3 
13.7 
13.3 
13.1 
13.1 
13.3 
13.8 
14.5 
16.5 
17.0 
17.0 
17.0 
15.4 
10.1 
4.8 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
1.0 
2.1 
4.8 
9.1 
14.2 
19.8 
25.5 
30.5 
34.8 
38.8 
42.9 
46.4 
48.3 
48.7 
48.5 
48.4 
48.2 
47.8 
47.0 
45.9 
44.9 
44.4 
44.3 
44.5 
45.1 
45.7 
46.0 
46.0 
46.0 
46.1 
46.7 
47.7 
48.9 
50.3 
51.6 
52.6 
53.0 
53.0 
52.9 
52.7 
52.6 
53.1 
54.3 
55.2 
55.5 
55.9 
56.3 
56.7 
56.9 
56.8 
56.0 
54.2 
52.1 
50.1 
47.2 
43.2 
39.2 
36.5 
34.3 
31.0 
26.0 
20.7 
15.4 
13.1 
12.0 
12.5 
14.0 
19.0 
23.2 
28.0 
32.0 
34.0 
36.0 
38.0 
40.0 
40.3 
40.5 
39.0 
35.7 
31.8 
27.1 
22.8 
21.1 
18.9 
18.9 
21.3 
23.9 
25.9 
28.4 
30.3 
30.9 
31.1 
31.8 
32.7 
33.2 
32.4 
28.3 
25.8 
23.1 
21.8 
21.2 
21.0 
21.0 
20.9 
19.9 
17.9 
15.1 
12.8 
12.0 
13.2 
17.1 
21.1 
21.8 
21.2 
18.5 
13.9 
12.0 
12.0 
13.0 
16.0 
18.5 
20.6 
22.5 
24.0 
26.6 
29.9 
34.8 
37.8 
40.2 
41.6 
41.9 
42.0 
42.2 
42.4 
42.7 
43.1 
43.7 
44.0 
44.1 
45.3 
46.4 
47.2 
47.3 
47.4 
47.4 
47.5 
47.9 
48.6 
49.4 
49.8 
49.8 
49.7 
49.3 
48.5 
47.6 
46.3 
43.7 
39.3 
34.1 
29.0 
23.7 
18.4 
14.3 
12.0 
12.8 
16.0 
19.1 
22.4 
25.6 
30.1 
35.3 
39.9 
44.5 
47.5 
50.9 
54.1 
56.3 
58.1 
59.8 
61.1 
62.1 
62.8 
63.3 
63.6 
64.0 
64.7 
65.2 
65.3 
65.3 
65.4 
65.7 
66.0 
65.6 
63.5 
59.7 
54.6 
49.3 
44.9 
42.3 
41.4 
41.3 
42.1 
44.7 
48.4 
51.4 
52.7 
53.0 
52.5 
51.3 
49.7 
47.4 
43.7 
39.7 
35.5 
31.1 
26.3 
21.9 
18.0 
17.0 
18.0 
21.4 
24.8 
27.9 
30.8 
33.0 
35.1 
37.1 
38.9 
41.4 
44.0 
46.3 
47.7 
48.2 
48.7 
49.3 
49.8 
50.2 
50.9 
51.8 
52.5 
53.3 
54.5 
55.7 
56.5 
56.8 
57.0 
57.2 
57.7 
58.7 
60.1 
61.1 
61.7 
62.3 
62.9 
63.3 
63.4 
63.5 
64.5 
65.8 
66.8 
67.4 
68.8 
71.1 
72.3 
72.8 
73.4 
74.6 
76.0 
76.6 
76.5 
76.2 
75.8 
75.4 
74.8 
73.9 
72.7 
71.3 
70.4 
70.0 
70.0 
69.0 
68.0 
68.0 
68.0 
68.1 
68.4 
68.6 
68.7 
68.5 
68.1 
67.3 
66.2 
64.8 
63.6 
62.6 
62.1 
61.9 
61.9 
61.8 
61.5 
60.9 
59.7 
54.6 
49.3 
44.9 
42.3 
41.4 
41.3 
42.1 
44.7 
48.4 
51.4 
52.7 
54.0 
57.0 
58.1 
59.2 
59.0 
59.1 
59.5 
60.5 
62.3 
63.9 
65.1 
64.1 
62.7 
62.0 
61.3 
60.9 
60.5 
60.2 
59.8 
59.4 
58.6 
57.5 
56.6 
56.0 
55.5 
55.0 
54.4 
54.1 
54.0 
53.9 
53.9 
54.0 
54.2 
55.0 
55.8 
56.2 
56.1 
55.1 
52.7 
48.4 
43.1 
37.8 
32.5 
27.2 
25.1 
26.0 
29.3 
34.6 
40.4 
45.3 
49.0 
51.1 
52.1 
52.2 
52.1 
51.7 
50.9 
49.2 
45.9 
40.6 
35.3 
30.0 
24.7 
19.3 
16.0 
13.2 
10.7 
8.8 
7.2 
5.5 
3.2 
1.1 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.8 
3.6 
8.6 
14.6 
20.0 
24.4 
28.2 
31.7 
35.0 
37.6 
39.7 
41.5 
43.6 
46.0 
48.4 
50.5 
51.9 
52.6 
52.8 
52.9 
53.1 
53.3 
53.1 
52.3 
50.7 
48.8 
46.5 
43.8 
40.3 
36.0 
30.7 
25.4 
21.0 
16.7 
13.4 
12.0 
12.1 
12.8 
15.6 
19.9 
23.4 
24.6 
25.2 
26.4 
28.8 
31.8 
35.3 
39.5 
44.5 
49.3 
53.3 
56.4 
58.9 
61.2 
62.6 
63.0 
62.5 
60.9 
59.3 
58.6 
58.6 
58.7 
58.8 
58.8 
58.8 
59.1 
60.1 
61.7 
63.0 
63.7 
63.9 
63.5 
62.3 
60.3 
58.9 
58.4 
58.8 
60.2 
62.3 
63.9 
64.5 
64.4 
63.5 
62.0 
61.2 
61.3 
62.6 
65.3 
68.0 
69.4 
69.7 
69.3 
68.1 
66.9 
66.2 
65.7 
64.9 
63.2 
60.3 
55.8 
50.5 
45.2 
40.1 
36.2 
32.9 
29.8 
26.6 
23.0 
19.4 
16.3 
14.6 
14.2 
14.3 
14.6 
15.1 
16.4 
19.1 
22.5 
24.4 
24.8 
22.7 
17.4 
13.8 
12.0 
12.0 
12.0 
13.9 
18.8 
25.1 
29.8 
33.8 
38.2 
43.4 
48.9 
53.8 
57.8 
61.5 
65.0 
68.4 
71.6 
73.0 
74.3 
76.2 
77.9 
79.5 
81.0 
82.3 
83.5 
84.6 
85.5 
86.3 
87.1 
88.1 
89.1 
90.1 
91.0 
91.7 
92.3 
92.8 
93.1 
93.1 
93.1 
93.1 
93.1 
93.1 
93.1 
93.1 
93.1 
93.1 
93.2 
93.2 
93.3 
93.7 
94.2 
95.0 
95.8 
96.4 
96.8 
97.0 
97.1 
97.2 
97.3 
97.4 
97.4 
97.4 
97.4 
97.3 
97.3 
97.3 
97.3 
97.2 
97.1 
97.0 
96.9 
96.7 
96.4 
96.1 
95.7 
95.5 
95.3 
95.2 
95.0 
94.9 
94.7 
94.5 
94.4 
94.4 
94.3 
94.3 
94.1 
93.9 
93.4 
92.8 
92.0 
91.3 
90.6 
90.0 
89.3 
88.7 
88.1 
87.4 
86.7 
86.0 
85.3 
84.7 
84.1 
83.5 
82.9 
82.3 
81.7 
81.1 
80.5 
79.9 
79.4 
79.0 
78.7 
78.7 
78.8 
79.1 
79.4 
79.6 
79.8 
79.8 
79.6 
79.3 
78.9 
78.5 
78.2 
77.9 
77.7 
77.7 
77.8 
77.9 
78.1 
78.3 
78.3 
78.4 
78.4 
78.4 
78.2 
78.0 
77.7 
77.3 
76.9 
76.6 
76.2 
75.7 
75.2 
74.7 
74.4 
74.3 
74.4 
74.6 
74.9 
75.1 
75.3 
75.5 
75.8 
75.9 
76.0 
76.0 
76.0 
75.9 
75.9 
75.8 
75.7 
75.5 
75.2 
75.0 
74.7 
74.1 
73.7 
73.3 
73.5 
74.0 
74.9 
76.1 
77.7 
79.2 
80.3 
80.8 
81.0 
81.0 
81.0 
81.0 
81.0 
80.9 
80.6 
80.3 
80.0 
79.9 
79.8 
79.8 
79.8 
79.9 
80.0 
80.4 
80.8 
81.2 
81.5 
81.6 
81.6 
81.4 
80.7 
79.6 
78.2 
76.8 
75.3 
73.8 
72.1 
70.2 
68.2 
66.1 
63.8 
61.6 
60.2 
59.8 
60.4 
61.8 
62.6 
62.7 
61.9 
60.0 
58.4 
57.8 
57.8 
57.8 
57.3 
56.2 
54.3 
50.8 
45.5 
40.2 
34.9 
29.6 
27.3 
29.3 
32.9 
35.6 
36.7 
37.6 
39.4 
42.5 
46.5 
50.2 
52.8 
54.3 
54.9 
54.9 
54.7 
54.1 
53.2 
52.1 
50.7 
49.1 
47.4 
45.2 
41.8 
36.5 
31.2 
27.6 
26.9 
27.3 
27.5 
27.4 
27.1 
26.7 
26.8 
28.2 
31.1 
34.8 
38.4 
40.9 
41.7 
40.9 
38.3 
35.3 
34.3 
34.6 
36.3 
39.5 
41.8 
42.5 
41.9 
40.1 
36.6 
31.3 
26.0 
20.6 
19.1 
19.7 
21.1 
22.0 
22.1 
21.4 
19.6 
18.3 
18.0 
18.3 
18.5 
17.9 
15.0 
9.9 
4.6 
1.2 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
2.2 
4.4 
6.3 
7.9 
9.2 
10.4 
11.5 
12.9 
14.7 
17.0 
19.8 
23.1 
26.7 
30.5 
34.1 
37.5 
40.6 
43.3 
45.7 
47.7 
49.3 
50.5 
51.3 
52.1 
52.7 
53.4 
54.0 
54.5 
55.0 
55.6 
56.3 
57.2 
58.5 
60.2 
62.3 
64.7 
67.1 
69.2 
70.7 
71.9 
72.7 
73.4 
73.8 
74.1 
74.0 
73.6 
72.5 
70.8 
68.6 
66.2 
64.0 
62.2 
60.9 
60.2 
60.0 
60.4 
61.4 
63.2 
65.6 
68.4 
71.6 
74.9 
78.4 
81.8 
84.9 
87.4 
89.0 
90.0 
90.6 
91.0 
91.5 
92.0 
92.7 
93.4 
94.2 
94.9 
95.7 
96.6 
97.7 
98.9 
100.4 
102.0 
103.6 
105.2 
106.8 
108.5 
110.2 
111.9 
113.7 
115.3 
116.8 
118.2 
119.5 
120.7 
121.8 
122.6 
123.2 
123.6 
123.7 
123.6 
123.3 
123.0 
122.5 
122.1 
121.5 
120.8 
120.0 
119.1 
118.1 
117.1 
116.2 
115.5 
114.9 
114.5 
114.1 
113.9 
113.7 
113.3 
112.9 
112.2 
111.4 
110.5 
109.5 
108.5 
107.7 
107.1 
106.6 
106.4 
106.2 
106.2 
106.2 
106.4 
106.5 
106.8 
107.2 
107.8 
108.5 
109.4 
110.5 
111.7 
113.0 
114.1 
115.1 
115.9 
116.5 
116.7 
116.6 
116.2 
115.2 
113.8 
112.0 
110.1 
108.3 
107.0 
106.1 
105.8 
105.7 
105.7 
105.6 
105.3 
104.9 
104.4 
104.0 
103.8 
103.9 
104.4 
105.1 
106.1 
107.2 
108.5 
109.9 
111.3 
112.7 
113.9 
115.0 
116.0 
116.8 
117.6 
118.4 
119.2 
120.0 
120.8 
121.6 
122.3 
123.1 
123.8 
124.4 
125.0 
125.4 
125.8 
126.1 
126.4 
126.6 
126.7 
126.8 
126.9 
126.9 
126.9 
126.8 
126.6 
126.3 
126.0 
125.7 
125.6 
125.6 
125.8 
126.2 
126.6 
127.0 
127.4 
127.6 
127.8 
127.9 
128.0 
128.1 
128.2 
128.3 
128.4 
128.5 
128.6 
128.6 
128.5 
128.3 
128.1 
127.9 
127.6 
127.4 
127.2 
127.0 
126.9 
126.8 
126.7 
126.8 
126.9 
127.1 
127.4 
127.7 
128.1 
128.5 
129.0 
129.5 
130.1 
130.6 
131.0 
131.2 
131.3 
131.2 
130.7 
129.8 
128.4 
126.5 
124.1 
121.6 
119.0 
116.5 
114.1 
111.8 
109.5 
107.1 
104.8 
102.5 
100.4 
98.6 
97.2 
95.9 
94.8 
93.8 
92.8 
91.8 
91.0 
90.2 
89.6 
89.1 
88.6 
88.1 
87.6 
87.1 
86.6 
86.1 
85.5 
85.0 
84.4 
83.8 
83.2 
82.6 
82.0 
81.3 
80.4 
79.1 
77.4 
75.1 
72.3 
69.1 
65.9 
62.7 
59.7 
57.0 
54.6 
52.2 
49.7 
46.8 
43.5 
39.9 
36.4 
33.2 
30.5 
28.3 
26.3 
24.4 
22.5 
20.5 
18.2 
15.5 
12.3 
8.7 
5.2 
0.0 
0.0 
0.0 
0.0 
0.0 
0.0 
];
% figure(101)
% subplot(2,1,1)
% plot(t,v,'k');
% title('Original WLTP Velocity Profile')
% xlabel('Time (s)')
% ylabel('Speed (km/h)');
% grid on
% m/s / 1000m * 1km *3600s / 1hr  
t = t/3;
v = v / 3.6; %km/h to m/s;
global maxS gear_ratio
ti = (0:0.01:t(end));
toDiv = maxS * gear_ratio * pi/30 * 0.0563 * 0.9;
v_norm = v / (max(v)/toDiv);
v_interpolated = interp1(t,v_norm,ti);
% plot(ti,v_interpolated);
pf(1,:) = ti;
pf(2,:) = v_interpolated;
vforsmallwheel = v / (max(v)/0.7065); 
pf(3,:) = interp1(t,vforsmallwheel,ti);
% subplot(2,1,2);
% plot(ti,v_interpolated,'b');
% title('Scaled Velocity Profile');
% xlabel('Time (s)')
% ylabel('Speed (m/s)');
% grid on

end