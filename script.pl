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

my $i;

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
	#######
	#######
	# while there's packages calculate their metrics
	$i = 0;
	while($packages[$i]){
		main_package("$packages[$i]");
		$i++;
	}
	#######
	#######
	$i = 0;	
	while($files_ids[$i]){
		main_file("$files_p[$i]/$files_name[$i]");
		$i++;
	}
}

sub main_file{
	my $file;
	my $loc_py = '0';
	my $loc_c = '0';
	
	$file = "/home/miguel/ros/repos@_";
	
	mkdir("temp_output",0777);

	#check of the file is a python or cpp file
	my $file_type = substr($file,-1);
	if($file_type eq "y"){
		system("radon raw $file >> temp_output/output_radon");
		$loc_py += python_metrics();
		insert_in_db("$files_ids[$i]", "0", "$loc_py","file_id","eco_FileMetrics");
	
	}
	else{
		system("cccc $file --outdir=./temp_output");
		$loc_c += cpp_metrics();
		insert_in_db("$files_ids[$i]", "1", "$loc_c","file_id","eco_FileMetrics");
	}
	system("rm -r temp_output");
}

# Connect to db and get all the files
sub get_files{
	my ($dsn, $connect, $query_handle, $query);
	my ($id,$pkg_id,$name,$path);

	# DATA SOURCE NAME
	$dsn = "dbi:mysql:$database:localhost:3306";

	# PERL DBI CONNECT
	$connect = DBI->connect($dsn, $user, $pw);

	# PREPARE THE QUERY
	$query = "SELECT * FROM ROSdb.eco_Files";
	$query_handle = $connect->prepare($query);

	# EXECUTE THE QUERY
	$query_handle->execute();

	# BIND TABLE COLUMNS TO VARIABLES
	$query_handle->bind_columns(undef, \$id, \$pkg_id ,\$name, \$path);
	# LOOP THROUGH RESULTS
	while($query_handle->fetch()) {
		$files_ids[$nr_files] = $id;
	   	$files_name[$nr_files] = $name;	
	   	$files_p[$nr_files++] = $path;
	}
}


####################
####################
####################
####################


sub main_package{
	my $package;
	my $loc_py = '0';
	my $loc_c = '0';
	if($ARGV[0]){ $package = "/home/miguel/ros/repos$ARGV[0]"; }
	else{ $package = "/home/miguel/ros/repos@_";}
	
	mkdir("temp_output",0777);

	# Run the programs to output the info about the package metrics
	run_radon($package);
	run_cccc($package);
	# Gather the info about the metrics from python and C files
	$loc_py += python_metrics();
	$loc_c += cpp_metrics();
	#print "Total number of Lines of Code: $loc\n";

	insert_in_db("$package_ids[$i]", "0", "$loc_py","pkg_id","eco_PackageMetrics");
	insert_in_db("$package_ids[$i]", "1", "$loc_c","pkg_id","eco_PackageMetrics");
	system("rm -r temp_output");
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
	print $cccc;
	
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
	$query = "SELECT * FROM ROSdb.eco_Packages";
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

########### Insert pkg metrics in DataBase #############
sub insert_in_db{
	my ($dsn, $connect, $query_handle, $query);
	
	# DATA SOURCE NAME
	$dsn = "dbi:mysql:$database:localhost:3306";

	# PERL DBI CONNECT
	$connect = DBI->connect($dsn, $user, $pw);

	# PREPARE THE QUERY
	$query = "INSERT INTO 
	ROSdb.$_[4] ($_[3],metric_id,value) 
	VALUES ('$_[0]','$_[1]', '$_[2]')";
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
        #print "Number of lines of code in the Python files: 0 (No Python files)\n";   
    }
    return $res;
}

########### Get Cpp Metrics ################
sub cpp_metrics {
    my $res = 0;
    # Check if file exists if not, print 0;
    my $filename = 'temp_output/cccc.xml';
    if (-e $filename) {
        # File Exists!
        my $metrics = XMLin('temp_output/cccc.xml');
        my $nr_books = 0;

        $res = $metrics->{'project_summary'}->{'lines_of_code'}->{'value'};
        print "Number of lines of code in the Cpp files: $res\n";
    }
    else{
        # File does not exist!
        print "Number of lines of code in the Cpp files: 0 (No C/Cpp files)\n";
    }
    return $res;
}    