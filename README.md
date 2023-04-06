# FN-McPAT
McPAT for FinFET-based and NCFET-based systems

----------------------------------------------------------------------------------
FN-CACTI
----------------------------------------------------------------------------------
1. Download FN-CACTI repository from https://github.com/marg-tools/FN-CACTI/
	Seperate repositories for FN-CACTI and FN-McPAT are maintained for ease of code maintenance
2. Follow the instructions provided in README.md for buulding FN-CACTI
3. fncacti folder will be created after the instructions are carefully followed
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
FN-McPAT
----------------------------------------------------------------------------------
1. Download FN-McPAT repository
	Extract the repository 
	Name the repository as fnmcpat
2. Copy fncacti folder into the fnmcpat folder
3. Sample config (.xml) files and outputs are provided in ProcessorDescriptionFiles folder

----------------------------------------------------------------------------------
Steps to build and run
----------------------------------------------------------------------------------
cd fnmcpat

1. To build executable
	 make clean
	 make
2. To execute 
     ./mcpat -infile ProcessorDescriptionFiles/<xxx>.xml
