#!/usr/bin/perl

use strict;
use warnings;
use Data::Dumper;
use Regexp::Common qw /comment/;

my $func = qr/[a-zA-Z0-9\*\:\<\_\>]+/; # i ignore case (Maisculas e Minusculas)
my $args = qr/(.*)/;
my $args2 = qr/[0-9a-zA-Z\,\s*\:\_\<\>\&\=\/]*/;

my %hash = ();
my %hash_depth = ();
my %lines = ();
my $cur_level = 0;
my $disc = 0;


if (($#ARGV) < 0) {
    print "No file inputed...\n";
    exit;
}
else{
    if (index($ARGV[0], ".hpp") != -1) {
      #print "HeaderFile\n";
      text_filter($ARGV[0]);
      #print $ARGV[0];
      print Dumper(\%hash);
    }
    else{ # Not an headerfile
      text_filter($ARGV[0]);
      #print $ARGV[0];
      print Dumper(\%hash);
    }

    if (%hash){
      open(my $fh, '>', 'info');
      for my $k (sort { $hash{$a}{line} <=> $hash{$b}{line} } keys %hash) {
          #print "$k\n\tLine: $hash{$k}{line}\n\tFunc: $hash{$k}{func}\n\tDepth: $hash{$k}{depth}\n";
          print $fh "$k\t$hash{$k}{depth}\n";
      }
      close $fh;
    }
    else{
      # Do nothing, hash is empty
    }
}

sub text_filter{
  my $filename = $_[0];
  my $line = 0;
  my $lines_jumped = 0;
  my $function;
  # Check if file exists if not, print 0;
  if (-e $filename) {
    # File Exists!
    open FILE, $filename or die "Couldn't open file: $!"; 
    while (<FILE>){
      chomp;
      s/^\s+|\s+$//g;
      # Remove coments
      s! ((['"]) (?: \\. | .)*? \2) | # skip quoted strings
         /\* .*? \*/ |  # delete C comments
         // [^\n\r]*   # delete C++ comments
       ! $1 || ' '   # change comments to a single space
       !xseg;    # ignore white space, treat as single line
       s/^'.*//;
       #s/($func)\s+($func)\(($args2)\)\s*\{/$1 $2\($3\)\{\n/;
       s/\h+/ /g; # remove multiple spacing
       s/($func)\s+($func)\s+\<\<\s*\(($args2)\)/$1 $2($3)/;
       s/($func)\s+($func)\s*\(($args2)\)\s*($func)/$1 $2($3)/;
       s/($func)\(\)\s\:\s($func)\(($args2)\)//; # Avoid struct definition to be consider as funtion: Data() : Text(void stuff)
       s/($func)\s+($func)\(($args2)\)\s*\{/$1 $2($3){\n/; # separates the 1 line function to 2 lines
       s/else\s*if/else-if/; # Fix a few bugs where else if were considered a function because it matched the params, instead of else if now reads else-if instead
       s/($func)\s+($func)\s+($func)\s*\(($args2)\)/$2 $3($4)/; # Remove the inline, private, etc before function definition
       s/if\s+\(/if(/; # Remove the space between if and the '(' to avoid some bugs
       s/($func)*\s*($func)\(($args2)\)\s*\:\s*($func)\(($args2)\)/$1$2($3):$4($5)/; #clear the white spaces in this exact function definition so it's not considered as a struct def.
       #print $_."\n";

       if($lines_jumped > 0){ $line += $lines_jumped; $lines_jumped = 0 }
       else{ $line++; }

       # Funtion with params separated by line
       if(($_ =~ m/($func)\s+($func)\(($args2)\s*\,/ ) && ($_ !~ m/\)/) )
       {
          my $f = $_;
          $f =~ s/($func)\s+($func)\(($args2)\,/$1 $2($3, /;
          my $nextline;
          do
          {
            $nextline = <FILE>; 
            chomp $nextline;
            if( $nextline =~ m/\)/ ){ $nextline =~ s/\s*($args2)\s*\)\s*\{*/$1)/; }
            $nextline =~ s/\s*($args2)\s*\,/$1, /;
            $f.=$nextline;
            $lines_jumped++;
          } while( $nextline !~ m/\)/ );
          $_ = $f;
          $lines_jumped++; # Since it's getting the next line is pertinent to increase the line itself;
        }

        if ( (($_ =~ m/($func)\s+($func)\(($args2)\)/) || ($_ =~ m/($func)*\s*($func)\(($args2)\):($func)\(($args2)\)/))# Expecific definition with : like void something() : anything() {}
             && ( ($_ !~ m/;/) || ($_ =~ m/($func)\s+($func)\(($args2)\)\s*\{/) )
             && ( $_ !~ m/[*,:,<,_,>]\s+($func)\(($args2)\)/ ) # sometimes a struct definition was considered a function
             )
        {
              # Functions defined in one single line, get the definition part
              if( ($_ =~ m/($func)\s+($func)\(($args2)\)\{/) || ($_ =~ m/($func)*\s*($func)\(($args2)\):($func)\(($args2)\)/) ){
                my @f = split(/{/, $_); 
                $_ = $f[0];
                #print "----->$_\n";
              }
              # clear the extra chars from the function like (void, int, chars, etc... and '{' if the function have it on the definition line )
              $_ =~ s/($func)\s+($func)\(($args2)\)[{]*/$2($3)/; 
              #print "Line: ".$line." FUNCTION:".$_."\n";
            if( (substr($_,-1) eq ')') && !(substr($_,0,1) eq '*') ){  
                $function = $_;
                $hash{ $function }{'func'} = 0;
                $hash{ $function }{'line'} = $line;
                $hash{ $function }{'depth'} = 0;
                $cur_level = 0;
            }
            else{
                print "Error: Not a funtion: $_\n";
            }
        }

        ###
        ##
        # FUNCTION CALLS
        ##
        ###
        if($function){
          if ( (($_ =~ m/($func)\(($args)\)\;/) || ($_ =~ m/([a-zA-Z\_]+)\<([a-zA-Z]+)\>\(([a-zA-Z\_]+)\)/)) && ($function)){
            #print "\t".$_."\n";
            $hash{ $function }{'func'}++;
          }
        }
        # END OF FUNTION CALLS

        ###
        ##
        # NESTING
        ##
        ###
        if( $function ){
          if ( ($_ =~ m/if\s*\(/) ){# else don't count as a level ($_ =~ m/else/) ){
            my $testline = <FILE>;
            chomp $testline;

            $cur_level++;
            if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }
            
            # ifs without the { }
            if( ($_ !~ m/\{/) && ($testline !~ m/\{/) ){ $cur_level--; }               
          }
          if ( ($_ =~ m/else\-if\s*\(/) ){
            $cur_level=$cur_level+2;
            if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }               
            
            my $testline = <FILE>;
            chomp $testline;
            # else-ifs without the { }
            if( ($_ !~ m/\{/) && ($testline !~ m/\{/) ){ $cur_level-=2; }            
          }
          # While Conditions, in case of having While, do while
          if ( $_ =~ m/while\s*\(/ ){
              $cur_level++;
              if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }
              
              my $testline = <FILE>;
              chomp $testline;
              if( ($_ !~ m/\{/) && ($testline !~ m/\{/) && ($_ !~ m/while\s*\((.*)\)\;/) ){
                  $cur_level--;
              }
          }
          if ( ($_ =~ m/do\s*\{/) ){
              my $testline = <FILE>;
              chomp $testline;
              if( $testline =~ m/\{/ ){
                $cur_level++;
                if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }            
              }
          }
          if ($_ =~ m/while\s*\((.*)\)\;/){
              $cur_level--;
          }
          # While Conditions, in case of having for
          if ( ($_ =~ m/for\s*\(/) ){
                $cur_level++;
                if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }              
                  # fors without the { }
                  my $testline = <FILE>;
                  chomp $testline;
                  if( ($_ !~ m/\{/) && ($testline !~ m/\{/) ){
                      $cur_level--;
                  }
          }
          # Try, Switchs don't count
          if (($_ =~ m/try/)){
              my $testline = <FILE>;
              chomp $testline;
                if( ($_ =~ m/\{/) || ($testline =~ m/\{/) ){
                    $disc++;
                    if($disc+$cur_level > $hash{$function}{'depth'}){$hash{$function}{'depth'} = $disc+$cur_level; }
                }
          }
          if ( ($_ =~ m/catch\s*\(/) || ($_ =~ m/switch\s*\(/) ){
              $disc++;
          }

          if ($_ =~ m/\}/){
              if($disc > 0){
                  $disc--;
              }
              else{
                  if($cur_level>0){ $cur_level--; }
              }
          }
      } ## END OF NESTING
      
    } # While END
    close FILE;
  } # If File exists END
} # Function END





###
###
### This function have been merged into the previous one
###
###
sub f_calls {
    my $function;
    my $filename = $_[0];
    my $line = 0;
    # Check if file exists if not, print 0;
    if (-e $filename) {
        # File Exists!
            open FILE, $filename or die "Couldn't open file: $!"; 
            while (<FILE>){
                chomp;
                s/^\s+|\s+$//g;
                # Remove coments
                s! ((['"]) (?: \\. | .)*? \2) | # skip quoted strings
                   /\* .*? \*/ |  # delete C comments
                   // [^\n\r]*   # delete C++ comments
                 ! $1 || ' '   # change comments to a single space
                 !xseg;    # ignore white space, treat as single line
                 s/^'.*//;
                 s/($func)\s+($func)\(($args2)\)\s*\{/$1 $2\($3\)\{\n/;
                 s/($func)\s+($func)\s+\<\<\s*\(($args2)\)/$1 $2($3)/;
                 s/($func)\s+($func)\s*\(($args2)\)\s*($func)/$1 $2($3)/;
                 s/($func)\(\)\s\:\s($func)\(($args2)\)//;
                 #print $_."\n";
                 
                 # Funtion with params separated by line
                 if(($_ =~ m/($func)\s+($func)\(($args2)\s*\,/ ) && ($_ !~ m/\)/) )
                 {
                    my $f = $_;
                    $f =~ s/($func)\s+($func)\(($args2)\,/$1 $2($3, /;
                    my $nextline;
                    do
                    {
                      $nextline = <FILE>; 
                      chomp $nextline;
                      if( $nextline =~ m/\)/ ){ $nextline =~ s/\s*($args2)\s*\)\s*\{*/$1)/; }
                      $nextline =~ s/\s*($args2)\s*\,/$1, /;
                      $f.=$nextline;
                    }while( $nextline !~ m/\)/ );
                    $_ = $f;
                 }

                $line++;
                if ( (($_ =~ m/($func)\s+($func)\(($args2)\)/) ) 
                     && ( ($_ !~ m/;/) || ($_ =~ m/($func)\s+($func)\(($args2)\)\s*\{/) )
                     )
                {
                      # Functions defined in one single line
                      if($_ =~ m/($func)\s+($func)\(($args2)\)\s*\{/){
                        $_ =~ s/($func)\s+($func)\(($args2)\)\s*\{([\=\_\;\}\s*0-9a-zA-Z])*/$1 $2($3)/;
                        #print "->$_\n";
                      }
                      # clear the extra chars from the function like (void, int, chars, etc...)
                      $_ =~ s/($func)\s+($func)\(($args2)\)[{]*/$2($3)/;
                      #print "Line: ".$line." FUNCTION:".$_."\n";
                    if( (substr($_,-1) eq ')') && !(substr($_,0,1) eq '*') ){  
                        $function = $_;
                        $hash{ $function }{'func'} = 0;
                        $hash{ $function }{'line'} = $line;
                    }
                    else{
                        print "Error: Not a funtion: $_\n";
                    }
                }
                if($function){
                  if ( (($_ =~ m/($func)\(($args)\)\;/) || ($_ =~ m/([a-zA-Z\_]+)\<([a-zA-Z]+)\>\(([a-zA-Z\_]+)\)/)) && ($function)){
                    #print "\t".$_."\n";
                    $hash{ $function }{'func'}++;
                  }
                }
            }   
            close FILE;
        }
    else{
        # File does not exist!  
    }
}
sub nesting {
    my $function;
    my $filename = $_[0];
    my $line = 0;
    # Check if file exists if not, print 0;
    if (-e $filename) {
        # File Exists!
            open FILE, $filename or die "Couldn't open file: $!"; 
            while (<FILE>){
                chomp;
                s/^\s+|\s+$//g;
                # Remove coments
                s! ((['"]) (?: \\. | .)*? \2) | # skip quoted strings
                   /\* .*? \*/ |  # delete C comments
                   // [^\n\r]*   # delete C++ comments
                 ! $1 || ' '   # change comments to a single space
                 !xseg;    # ignore white space, treat as single line
                 s/^'.*//;
                 s/($func)\s+($func)\(($args2)\)\s*\{/$1 $2\($3\)\{\n/;
                 s/($func)\s+($func)\s+\<\<\s*\(($args2)\)/$1 $2($3)/;
                 s/($func)\s+($func)\s*\(($args2)\)\s*($func)/$1 $2($3)/;
                 s/($func)\(\)\s\:\s($func)\(($args2)\)//;
                   
                 # Funtion with params separated by line
                 if(($_ =~ m/($func)\s+($func)\(($args2)\s*\,/ ) && ($_ !~ m/\)/) )
                 {
                    my $f = $_;
                    $f =~ s/($func)\s+($func)\(($args2)\,/$1 $2($3, /;
                    my $nextline;
                    do
                    {
                      $nextline = <FILE>; 
                      chomp $nextline;
                      if( $nextline =~ m/\)/ ){ $nextline =~ s/\s*($args2)\s*\)\s*\{*/$1)/; }
                      $nextline =~ s/\s*($args2)\s*\,/$1, /;
                      $f.=$nextline;
                    }while( $nextline !~ m/\)/ );
                    $_ = $f;
                 }

                 
                $line++;
                if ( (($_ =~ m/($func)\s+($func)\(($args2)\)/) ) 
                     && ( ($_ !~ m/;/) || ($_ =~ m/($func)\s+($func)\(($args2)\)\s*\{/) )
                     )
                {
                      # Functions defined in one single line
                      if($_ =~ m/($func)\s+($func)\(($args2)\)\s*\{/){
                        $_ =~ s/($func)\s+($func)\(($args2)\)\s*\{([\=\_\;\}\s*0-9a-zA-Z])*/$1 $2($3)/;
                        #print "->$_\n";
                      }
                      # clear the extra chars from the function like (void, int, chars, etc...)
                      $_ =~ s/($func)\s+($func)\(($args2)\)[{]*/$2($3)/;
                      #print "Line: ".$line." FUNCTION:".$_."\n";
                      if( (substr($_,-1) eq ')') && !(substr($_,0,1) eq '*') ){   
                          $function = $_;
                          $hash{$function}{'depth'} = 0;
                          $cur_level = 0;
                       }
                       else{
                          print "Error: Not a funtion: $_\n";
                       }
                }
                if( $function ){
                    # If Conditions, in case of having if, else, else if
                    if ( ($_ =~ m/if\s*\(/) ){# else don't count as a level ($_ =~ m/else/) ){
                      my $testline = <FILE>;
                      chomp $testline;

                      $cur_level++;
                      if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }
                      
                      # ifs without the { }
                      if( ($_ !~ m/\{/) && ($testline !~ m/\{/) ){ $cur_level--; }               
                    }
                    if ( ($_ =~ m/else\sif\s*\(/) ){
                      $cur_level=$cur_level+2;
                      if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }               
                    }
                    # While Conditions, in case of having While, do while
                    if ( $_ =~ m/while\s*\(/ ){
                        $cur_level++;
                        if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }
                        
                        my $testline = <FILE>;
                        chomp $testline;
                        if( ($_ !~ m/\{/) && ($testline !~ m/\{/) && ($_ !~ m/while\s*\((.*)\)\;/) ){
                            $cur_level--;
                        }
                    }
                    if ( ($_ =~ m/do\s*\{/) ){
                        my $testline = <FILE>;
                        chomp $testline;
                        if( $testline =~ m/\{/ ){
                          $cur_level++;
                          if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }            
                        }
                    }
                    if ($_ =~ m/while\s*\((.*)\)\;/){
                        $cur_level--;
                    }
                    # While Conditions, in case of having for
                    if ( ($_ =~ m/for\s*\(/) ){
                          $cur_level++;
                          if($disc+$cur_level > $hash{$function}{'depth'}){ $hash{$function}{'depth'} = $disc+$cur_level; }              
                            # fors without the { }
                            my $testline = <FILE>;
                            chomp $testline;
                            if( ($_ !~ m/\{/) && ($testline !~ m/\{/) ){
                                $cur_level--;
                            }
                    }
                    # Try, Switchs don't count
                    if (($_ =~ m/try/)){
                        my $testline = <FILE>;
                        chomp $testline;
                          if( ($_ =~ m/\{/) || ($testline =~ m/\{/) ){
                              $disc++;
                              if($disc+$cur_level > $hash{$function}{'depth'}){$hash{$function}{'depth'} = $disc+$cur_level; }
                          }
                    }
                    if ( ($_ =~ m/catch\s*\(/) || ($_ =~ m/switch\s*\(/) ){
                        $disc++;
                    }

                    if ($_ =~ m/\}/){
                        if($disc > 0){
                            $disc--;
                        }
                        else{
                            if($cur_level>0){ $cur_level--; }
                        }
                    }
                }
            }   
            close FILE;
        }
    else{
        # File does not exist!  
    }
    #$hash{$function}{'depth'} = $hash{$function}{'depth'} - $disc;
}
