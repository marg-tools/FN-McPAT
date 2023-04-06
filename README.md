# FN-McPAT
McPAT for FinFET-based and NCFET-based systems

----------------------------------------------------------------------------------
FN-CACTI
----------------------------------------------------------------------------------
1. Download FN-CACTI repository from https://github.com/marg-tools/FN-CACTI/
2. Seperate repositories for FN-CACTI and FN-McPAT are maintained for ease of code maintenance
3. Follow the instructions provided in README.md for building FN-CACTI
4. fncacti folder will be created after the instructions are carefully followed
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
FN-McPAT
----------------------------------------------------------------------------------
1. Download FN-McPAT repository
2. Extract the repository 
3. Name the repository as fnmcpat
4. Copy fncacti folder into the fnmcpat folder
5. Sample config (.xml) files and outputs are provided in ProcessorDescriptionFiles folder

----------------------------------------------------------------------------------
Steps to build and run
----------------------------------------------------------------------------------
cd fnmcpat

1. To build executable

	 make clean

	 make
	 
2. To execute 
     ./mcpat -infile ProcessorDescriptionFiles/<xxx>.xml
