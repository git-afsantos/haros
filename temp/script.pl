#!/usr/bin/perl
# '0', 'LOC_PY', 'Number of Lines of Code (Python)'
# '1', 'LOC_CPP', 'Number of Lines of Code (C++)'
# '2', 'COM', 'Number of Lines of Comments'
# '3', 'CC', 'Cyclomatic Complexity'
# '4', 'FC', 'Number of Function Calls'
# '5', 'MNC', 'Maximum Nesting of Control'
# '6', 'ESP', 'Estimated Static Path count'
# '7', 'CBO', 'Coupling between objects'
# '8', 'NOC', 'Number of Children'
# '9', 'WMC', 'Weighted Methods per Class'
# '10', 'DIT', 'Depth of Inheritance Tree'
# '11', 'MAC', 'Number of Methods Available in Class'

## File-Based -> 
# 1 - Lines of Code
# 2 - Comment Lines
## Function-Based  
# 1 - Lines of Code - Executable lines
# 3 - Cyclomatic Complexity
# 4 - Number of function calls
# 5 - Maximum nesting of control
# 6 - Estimated static path count
##
# Class-Based
# 7 -> Coupling between objects
# 8 -> Number of immediate Children
# 9 -> Weighted Methods per Class
# 10 -> Deepest level if inheritance
# 11 -> Number of Methods Available in Class

#!/usr/bin/perl


use strict;
use XML::Simple;
use Data::Dumper;
use warnings;
use DBI;
use DBD::mysql;

# CONFIG VARIABLES
my $platform = "mysql";
my $database = "ROSdb";
my $host = "localhost";
my $port = "3306";
my $user = "root";
my $pw = "password";
my $home = "/home/miguel/ros/repos";

my $id;

my @packages;
my @package_ids;
my $nr_packages = 0;

my @files_p;
my @files_name;
my @files_ids;
my $nr_files = 0;

main();

sub main{
	# get the list of packages
	get_packages();
	get_files();

	# while there's packages calculate their metrics
	$id = 0;
	#while($packages[$id]){
	#while($id<20){
	#	main_package("$packages[$id]");
	#	$id++;
	#}
	
	$id = 0;	
	while($files_ids[$id]){
	#while($id<1){
		main_file("$files_p[$id]/$files_name[$id]");
		$id++;
	}
}

sub main_file{
	my $file;
	my $loc_py = '0';
	my $loc_c = '0';
	
	$file = "$home@_";
	
	mkdir("temp_output",0777);
	
	#check if the file is a python or cpp file
	my $file_type = substr($file,-1);
	if($file_type eq "y"){
		system("radon raw $file >> temp_output/output_radon");
		$loc_py = python_metrics();
		insert_in_db("'$files_ids[$id]','0','$loc_py'","file_id,metric_id,value","haros_File_Metrics");
	}
	else{
		system("cccc $file --outdir=./temp_output");
		my @loc_c = cpp_metrics();
		print "LOC: $loc_c[0]\nComments: $loc_c[1]\n";
		insert_in_db("'$files_ids[$id]','1','$loc_c[0]'","file_id,metric_id,value","haros_File_Metrics");
		insert_in_db("'$files_ids[$id]','2','$loc_c[1]'","file_id,metric_id,value","haros_File_Metrics");
		function_based_metrics();
	}
	system("rm -r temp_output");
}

sub main_package{
	my $package;
	my $loc_py = '0';
	my @loc_c = ('0','0');
	
	$package = "$home@_";
	
	mkdir("temp_output",0777);

	# Run the programs to output the info about the package metrics
	run_radon($package);
	run_cccc($package);
	# Gather the info about the metrics from python and C files
	$loc_py = python_metrics("package_id,metric_id,value","haros_Package_Metrics","$package_ids[$id]");
	insert_in_db("'$package_ids[$id]','0','$loc_py'","package_id,metric_id,value","haros_Package_Metrics");
	@loc_c = cpp_metrics();
	insert_in_db("'$package_ids[$id]','1','$loc_c[0]'","package_id,metric_id,value","haros_Package_Metrics");
	
	system("rm -r temp_output");
}


####################
####################
####################
####################

sub function_based_metrics{
	my $i=0;
	my @result = qx{find temp_output/ -type f -name "*.xml" ! -iname "cccc.xml"};
	while($result[$i]){
		chop($result[$i]);
		my @funcs = split /\//, $result[$i];
		func_based_aux("$funcs[1]");
		$i++;
	}
}

sub func_based_aux{	
	my ($cyclo_complex,$loc,$locmm,$cbo,$noc,$wmc,$dit,$mac,$func_params,$line)=-1;

	my @module_name = split(/\./,$_[0]);

	my $filename = "temp_output/".$_[0];
	 if (-e $filename) {
        my $xml = XMLin($filename);
  
        $cbo = $xml->{'module_summary'}->{'coupling_between_objects'}->{'value'};
        $noc = $xml->{'module_summary'}->{'number_of_children'}->{'value'};
        $wmc = $xml->{'module_summary'}->{'weighted_methods_per_class_unity'}->{'value'};
        $dit = $xml->{'module_summary'}->{'depth_of_inheritance_tree'}->{'value'};
        $mac = $xml->{'module_summary'}->{'weighted_methods_per_class_visibility'}->{'value'};
        print "Module Name : $module_name[0]\n";
        print "Coupling Between Objects: $cbo\nNumber_of_children: $noc\nweighted_methods_per_class: $wmc\ndepth_of_inheritance_tree: $dit\nweighted_methods_per_class_visibility: $mac\n\n";
    	### File Class Metrics
    	insert_in_db("'$files_ids[$id]','$module_name[0]','7','','$cbo'","file_id,class_name,line,metric_id,value","haros_File_Class_Metrics");
		insert_in_db("'$files_ids[$id]','$module_name[0]','8','','$noc'","file_id,class_name,line,metric_id,value","haros_File_Class_Metrics");
		insert_in_db("'$files_ids[$id]','$module_name[0]','9','','$wmc'","file_id,class_name,line,metric_id,value","haros_File_Class_Metrics");
		insert_in_db("'$files_ids[$id]','$module_name[0]','10','','$dit'","file_id,class_name,line,metric_id,value","haros_File_Class_Metrics");
		insert_in_db("'$files_ids[$id]','$module_name[0]','11','','$mac'","file_id,class_name,line,metric_id,value","haros_File_Class_Metrics");

		my (@func_name, @params);
		my $i = 0;
		
		#print $filename."\n"; print Dumper( $xml->{'procedural_detail'}->{'member_function'}); getc();
		
		## Have only 1 function
		if(my $name = $xml->{'procedural_detail'}->{'member_function'}->{'name'}){
			@func_name = split(/\(/,$name);
			@params = split(/\,/,$name);

			my $aux = $name;
			$aux =~ s/[\(]/,/g;
			$aux =~ s/[\)\s*]//g;
			@params = split(/\,/,$aux);
			$cyclo_complex = $xml->{'procedural_detail'}->{'member_function'}->{'McCabes_cyclomatic_complexity'}->{'value'};
	        $loc = $xml->{'procedural_detail'}->{'member_function'}->{'lines_of_code'}->{'value'};
	        $locmm = $xml->{'procedural_detail'}->{'member_function'}->{'lines_of_comment'}->{'value'};
    		#$line = $xml->{'procedural_detail'}->{'member_function'}->{'extent'}->{'source_reference'}->{'line'};
    		$func_params = (scalar @params)-1;
    		print "Function Name: $func_name[0]\n";
			print "McCabes Cyclomatic Complexity: $cyclo_complex\nLines of Code: $loc\nLines of Comments: $locmm\nFunction Params: $func_params\n";
			#### WARNING:: Change $keys[$i] to $func_name[0] to appear just the function name and not the parameters
			### File Function Metrics 
	    	insert_in_db("'$files_ids[$id]','$name','','1','$loc'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
	    	insert_in_db("'$files_ids[$id]','$name','','2','$locmm'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
	    	insert_in_db("'$files_ids[$id]','$name','','3','$cyclo_complex'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
	    	insert_in_db("'$files_ids[$id]','$name','','12','$func_params'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
			    	
		}
		else{ ## 0 or more functions
				my @keys = keys %{ $xml->{'procedural_detail'}->{'member_function'} };
    			my @values = values $xml->{'procedural_detail'}->{'member_function'};
    	    	while($keys[$i]){
		    	
					@func_name = split(/\(/,$keys[$i]);
					@params = split(/\,/,$keys[$i]);
					
					my $aux = $keys[$i];
					$aux =~ s/[\(]/,/g;
					$aux =~ s/[\)\s*]//g;
					@params = split(/\,/,$aux);

					$cyclo_complex = $values[$i]->{'McCabes_cyclomatic_complexity'}->{'value'};
			        $loc = $values[$i]->{'lines_of_code'}->{'value'};
			        $locmm = $values[$i]->{'lines_of_comment'}->{'value'};

		    		#if (exists $values[$i]->{'extent'}->{'source_reference'}->{'line'}){
		    		#	$line = $values[$i]->{'extent'}->{'source_reference'}->{'line'};
		    		#}
		    		#else{$line = $values[$i]->{'extent'}[0]->{'source_reference'}->{'line'};}
		    		$func_params = (scalar @params)-1;
		    		print "Function Name: $func_name[0]\n";
					print "McCabes Cyclomatic Complexity: $cyclo_complex\nLines of Code: $loc\nLines of Comments: $locmm\nFunction Params: $func_params\n";
					#### WARNING:: Change $keys[$i] to $func_name[0] to appear just the function name and not the parameters
					### File Function Metrics 
			    	insert_in_db("'$files_ids[$id]','$keys[$i]','','1','$loc'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
			    	insert_in_db("'$files_ids[$id]','$keys[$i]','','2','$locmm'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
			    	insert_in_db("'$files_ids[$id]','$keys[$i]','','3','$cyclo_complex'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
			    	insert_in_db("'$files_ids[$id]','$keys[$i]','','12','$func_params'","file_id,function_name,line,metric_id,value","haros_File_Function_Metrics");
			    	$i++;
				}
			}
		}
    else
    {
       print "File does not exist!\n";
    }
}

sub run_radon{
	system("radon raw @_ >> temp_output/output_radon");
}

sub run_cccc{
	my $cccc;
	my $i=0;
	my @result = qx{find @_ -name '*.cpp' -o -name '*.h'};
	while($result[$i]){
		chop($result[$i]);
		$cccc .= "$result[$i] ";		
		$i++;
	}

	system("cccc $cccc--outdir=./temp_output");
}

# Connect to db and get all the packages
sub get_packages{
	my ($dsn, $connect, $query_handle, $query);
	my ($id,$name,$isMetapackage,$description,$wiki,$git,$branch,$path);

	# DATA SOURCE NAME
	$dsn = "dbi:mysql:$database:localhost:3306";

	# PERL DBI CONNECT
	$connect = DBI->connect($dsn, $user, $pw);

	# PREPARE THE QUERY
	$query = "SELECT * FROM ROSdb.haros_Packages";
	$query_handle = $connect->prepare($query);

	# EXECUTE THE QUERY
	$query_handle->execute();

	# BIND TABLE COLUMNS TO VARIABLES
	$query_handle->bind_columns(undef, \$id, \$name, \$isMetapackage, \$description, \$wiki, \$git, \$branch, \$path);
	# LOOP THROUGH RESULTS
	while($query_handle->fetch()) {
		$package_ids[$nr_packages] = $id if $path;
	   	$packages[$nr_packages++] = $path if $path; #Remove the nulls
	} 
}

# Connect to db and get all the files
sub get_files{
	my ($dsn, $connect, $query_handle, $query);
	my ($id,$package_id,$name,$path);

	# DATA SOURCE NAME
	$dsn = "dbi:mysql:$database:localhost:3306";

	# PERL DBI CONNECT
	$connect = DBI->connect($dsn, $user, $pw);

	# PREPARE THE QUERY
	$query = "SELECT * FROM ROSdb.haros_Files";
	$query_handle = $connect->prepare($query);

	# EXECUTE THE QUERY
	$query_handle->execute();

	# BIND TABLE COLUMNS TO VARIABLES
	$query_handle->bind_columns(undef, \$id, \$package_id ,\$name, \$path);
	# LOOP THROUGH RESULTS
	while($query_handle->fetch()) {
		$files_ids[$nr_files] = $id;
	   	$files_name[$nr_files] = $name;	
	   	$files_p[$nr_files++] = $path;
	}
}

########### Insert pkg metrics in DataBase #############
sub insert_in_db{
	my ($dsn, $connect, $query_handle, $query);
	
	# DATA SOURCE NAME
	$dsn = "dbi:mysql:$database:localhost:3306";

	# PERL DBI CONNECT
	$connect = DBI->connect($dsn, $user, $pw);

	# PREPARE THE QUERY
	$query = "INSERT INTO 
	ROSdb.$_[2] ($_[1]) 
	VALUES ($_[0])";
	$query_handle = $connect->prepare($query);
	# EXECUTE THE QUERY
	$query_handle->execute();
}

########### Get Python Metrics #############
sub python_metrics {
    my $token = qr/[ a-zA-Z\_\/\.\:]+/; # i ignore case (Maisculas e Minusculas)
	my $token1 = qr/[a-zA-Z\_\/]+\//;
    my $res = '0';
    # Check if file exists if not, print 0;
    my $filename = 'temp_output/output_radon';
    if (-e $filename) {
        # File Exists!
            open FILE, "temp_output/output_radon" or die "Couldn't open file: $!"; 
            while (<FILE>){
                s/\s*//;
                s/\/([a-zA-Z0-9\_\.\-\/])*\n//;
                #s/($token1)*(\w+)\.(\w+)\n//;
                s/LLOC\: (\d+)\n//; #Ignora o LLOC
                s/SLOC\: (\d+)\n//; #Ignora o SLOC
                s/LOC\: (\d+)\n/$1/;
                s/Comments\: (\d+)\n//; #Ignora os Comments
                s/Multi\: (\d+)\n//; #Ignora o Multi
                s/Blank\: (\d+)\n//; #Ignora os Blank Spaces
                s/\-\sComment\sStats\n//; #Ignora os Comment Stats
                s/\(($token)\s\%\s($token)\)\:\s(\d+)\%\n//;
                s/\(($token)\s\+\s($token)\s\%\s($token)\)\:\s[0-9]+\%\n\s*//;
                #print "$_\n";
                if(!($_ eq "")){
                    $res = $res + $_;
                    #print "->$_<-\n";
                }
            }
            close FILE;
            #print "Number of lines of code in the Python files: $res\n";
    }
    else{
        # File does not exist!
        print "Number of lines of code in the Python files: 0 (No Python files)\n";   
    }
    return $res;
}

########### Get Cpp Metrics ################
sub cpp_metrics {
    my @res;
    my $loc = 0;
    my $com = 0;
    # Check if file exists if not, print 0;
    my $filename = 'temp_output/cccc.xml';
    if (-e $filename) {
        # File Exists!
        my $metrics = XMLin('temp_output/cccc.xml');

        $loc = $metrics->{'project_summary'}->{'lines_of_code'}->{'value'};
        $com = $metrics->{'project_summary'}->{'lines_of_comment'}->{'value'};
    	#print "Number of lines of code in the Cpp file(s): $res\n";
    	#print "Number of lines of comments in the Cpp file(s): $com\n";
    }
    else{
        # File does not exist!
        print "Number of lines of code in the Cpp files: 0 (No C/Cpp files)\n";
    }

    $res[0]=$loc;
    $res[1]=$com;

    return @res;
}    