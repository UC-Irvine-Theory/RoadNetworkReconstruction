```data.zip``` contains a sample of the data used as input, based on datasets obtained from the 9th [[1]](#1) and 10th [[2]](#2) DIMACS Implementation Challenges.

The two source files ```reconstruct.cpp``` and ```reconstruct_ESA.cpp``` respectively contain implementations for our algorithm and the algorithm 
introduced by Mathieu & Zhou [[3]](#3).

To compile, run
```make```

Then, for example, to reconstruct ```HI.tmp``` (located in the ```data``` directory), run
```./reconstruct HI```
or
```./reconstruct_ESA HI```

which will store the results in the directories ```out``` and ```out_ESA``` respectively.

<a id="1">[1]</a> 
http://www.diag.uniroma1.it/~challenge9/data/tiger

<a id="2">[2]</a> 
David A. Bader, Henning Meyerhenke, Peter Sanders, and Dorothea Wagner, editors. Graph Partitioning and Graph Clustering, 10th DIMACS Implementation Challenge Workshop,
Georgia Institute of Technology, Atlanta, GA, USA, February 13-14, 2012. Proceedings, volume 588 of Contemporary Mathematics. American Mathematical Society, 2013. doi:10.1090/conm/588.

<a id="3">[3]</a> 
Claire Mathieu and Hang Zhou. A simple algorithm for graph reconstruction. In 29th Annual European Symposium on Algorithms (ESA 2021), 2021.
