# Capacity Vehicule Routing Problem - CVRP

### Requirements
- Python 3.5.2

#### Packages Installation
- matplotlib v3.0.2 - [License](https://matplotlib.org/users/license.html)
- ortools v6.9.5824 - [License](https://github.com/google/or-tools/blob/master/LICENSE-2.0.txt)
- argparse v1.4.0 - [License](https://github.com/bewest/argparse/blob/master/LICENSE.txt)

To installs the packages: `pip install - r requirements.txt`
Or `pip3 install - r requirements.txt`

#### Input File
The input consists of `N + 1` lines. The first line contains 3 numbers: The number of customers `N`, the number of vehicles `V`, and the vehicle capacity `c`. It is followed by `N` lines, each line represents
a​ ​ location​ ​ triple​ ​ `<di;​ ​ xi;​ ​ yi>​` ​ with​ ​ a ​ ​ demand​ ​ `di​` ​ and​ ​ a ​ ​ point​ ​ `xi`;​ ​ `yi`.


| N | V | c |
| --- | --- | --- |
| d_0 | x_0 | y_0 |

4 examples files are included for quick tests: `input-20.tsv`,`input-100.tsv`,`input-500.tsv`,`input-2000.tsv`.

#### Output File

The output has `V+1` lines. The first line contains two values `obj` and `opt`. `obj` is the length of all of the vehicle routes (i.e. the objective value) as a real number. `opt` should be `1` if your algorithm proved optimality and `0` otherwise.
The following `V` lines represent the vehicle routes `T` encoding
the solution. Each vehicle line starts with warehouse identifier 0 followed by the identifiers of the customers serviced by that vehicle and ends with the warehouse identifier `0`.

| obj | opt |
| --- | --- | 
| 0 | 0 t_0_1 t_0_2 ... 0 |

#### Usage

```
usage: take_home_assignment.py [-h] [-v] [-c] [-t MAX_TIME] input_file output_file

positional arguments:
  input_file            Path to input file
  output_file           Path to output file

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         Write the solution details in the console
  -c, --chart           Display a Visualisation chart of the solution
  -t MAX_TIME, --max_time MAX_TIME
                        Max time in seconds for optimization [default 60s]
```


