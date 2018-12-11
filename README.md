# EleNa: Elevation-based Navigation

The usual Navigation System tries to get the shortest or the fastest path from one point to another. However, in some cases, having the ability to further customize the path is desirable. Such a case could arise if, while working out, following a path that maximizes or minimizes the elevation gain is more convenient than the usual shortest path in order to get better training results. Therefore, the EleNa project tries to find the best path between two points according to the user preferences regarding elevation gain and some distance constraint with respect to the shortest path.

## Getting Started
### Prerequisites
- Python 3.7
- OSMnx for python 3.6 or above

### Installing
- Clone the following repository using the following command:
`git clone https://github.com/aish-anand/EleNA-520.git`
- Install the OSMnx library:
	* with pip - `pip install osmnx`
	* with conda - `conda install -c conda-forge osmnx`
	It is recommended to create a new virtual environment to install the packages.
- Once the repository is cloned, simply navigate to the repository's directory and run the following command with the API key value provided in the report
`python Main.py --algorithm ALGORITHM VALUE --key APIKEY`
## Running the tests
- The test will accept amount of extra travel and number of destinations for testing as the input and generate graphs comparing the total length and elevation of routes by using the different algorithms. 
- To run the 
test, use the following command:
`python Test.py`
## Built With
Python 3.7 and OSMnx
## Versioning
Git
## Authors
Dhanya Bhat, Ramya Sarma, Aishwarya Turuvekere