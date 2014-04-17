#!/usr/bin/perl -w

# @author Komei Sugiura(komei.sugiura\@gmail.com)
# @version 0.1

use strict;

# use Data::Dumper; $Data::Dumper::Terse = 1; $Data::Dumper::Indent = 1;

&main();


sub main
{
   my $filename = "GPSR1.gram";	# load grammar from this file
   my $max_generation = 200;	# max number of sentences

   # $verb_phrase_templates_db = e.g. '$grasp_it' => ['take it', 'grasp it' ],
   my $verb_phrase_templates_db = &load_verb_pharase_templates($filename);
   
   # $sentence_templates_db = e.g. [  '$goto, $find_person, and $fetch',  '$ask_name, $follow_him, and $exit']
   my $sentence_templates_db = &load_sentetence_templates($filename);


#    print Dumper $verb_phrase_templates_db;
#    print Dumper $sentence_templates_db;
   my $generated_sentences = &generate_sentences($verb_phrase_templates_db, 
						 $sentence_templates_db, 
						 $max_generation);
   &display_sentence($generated_sentences);
}


sub display_sentence
{
   my ($generated_sentences) = @_;

   foreach my $s (@$generated_sentences){
      next if $s =~ /\$\w+/;	# rewriting failed
      $s = ucfirst $s . ".";
      print "$s\n";
   }
}


# @param: $verb_phrase_templates = reference of an hash of defined VPs
# @param: $sentence_templates = reference of an array of sentences
# @param: $max_rewriting = max number of rewriting
# @brief: rewriting defined sentences
# @return: generated sentences
sub generate_sentences
{
   my ($verb_phrase_templates, $sentence_templates, $max_rewriting) = @_;
   
   my (@generated) = @$sentence_templates;

   for( my $counter = 0; $counter < $max_rewriting; $counter++){
#       print Dumper \@generated;
      # get one sentence from the array and rewrite it
      my $target = shift @generated;
      if( $target !~ /\$\w+/){
	 push @generated, $target;
	 next;
      }

      my ($before, $matched, $after) = ($`, $&, $');

      # if matched index is not defined, delete the index
      if ( ! defined $verb_phrase_templates->{$matched} ){
	 print "Warning: ID:$matched is not defined\n";
	 push @generated, "$before$after";
      }

      # rewrite sentence temlate by defined VP
      # and put the rewritten sentences into the array
      foreach my $VP ( @{ $verb_phrase_templates->{$matched} } ){
	 push @generated, "$before$VP$after";
      }
   }

   return \@generated;
}


# @return e.g. [  '$goto, $find_person, and $fetch',  '$ask_name, $follow_him, and $exit']
sub load_sentetence_templates
{
   my ($filename) = @_;
   my $lines = &read_file($filename);
   my @out;			# sentence-template DB

   foreach my $line (@$lines){
      next if $line !~ /\[(.+)\]/;
      push @out, $1;
   }
   return \@out;
}


# @return e.g. '$grasp_it' => ['take it', 'grasp it' ],
sub load_verb_pharase_templates
{
   my ($filename) = @_;
   my $lines = &read_file($filename);

   my %out;			# verb-phrase-template DB

   foreach my $line (@$lines){
      # parse a sentence like "$find_person = find a person"
      next if $line !~ /^\s*(\$.+)=(.+)$/;
      my ($VP_id, $VP_sentences) = ($1, $2);

      my (@verb_pharases) = split /\|/, $VP_sentences;

      # delete unnecessary spaces
      $VP_id =~ s/\s+//g;
      s/^\s+//g foreach @verb_pharases;
      s/\s+$//g foreach @verb_pharases;

      $out{$VP_id} = \@verb_pharases;
   }
   return \%out;
}


# @brief read file
# @param filename 
# @return a reference of an array of all lines
sub read_file
{
   my ($filename) = @_;
   my @out;
   open (FILE, "<$filename") or die "cannot open $filename\n";

   while(<FILE>){
      chomp;
      next if ! $_;
      next if /^#/;
      push @out, $_;
   }
   close FILE;
   return \@out;
}
