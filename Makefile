all: reconstruct reconstruct_ESA

reconstruct:
	g++ -std=c++17 -o reconstruct reconstruct.cpp
reconstruct_ESA:
	g++ -std=c++17 -o reconstruct_ESA reconstruct_ESA.cpp
