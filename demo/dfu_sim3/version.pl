my $git_version = `git describe --dirty --always --tags`;
$git_version =~ s/\s+//g;

print "Got version!";

my $string =<<EOF;
#include "freakusb.h"
#include "hw.h"

U8 serial_str_desc[] PROGMEM =
{
    0x%x,       // bLength: 0x14 (20 bytes = sizeof str fields + string)
    STR_DESCR,  // bDescriptorType: String
                // Serial: Beta 0.50
#if defined( PCB_V7 )
    'V',0,'0',0,'7',0,'-',0,
#elif defined( PCB_V8 )
    'V',0,'0',0,'8',0,'-',0,
#elif defined( PCB_V10 )
    'V',0,'1',0,'0',0,'-',0,
#else
    #error "No PCB Revision Selected"
#endif
    %s
};
EOF

$git_version_expanded = $git_version;
$git_version_expanded =~ s/(.{1,1})/\'$1\'\,0\,/gs;

my $filename = "version.c";
open(my $fh, '>', $filename) or die "Could not open file '$filename' $!";
printf $fh $string,length($git_version)*2+8+2, $git_version_expanded;
close $fh;