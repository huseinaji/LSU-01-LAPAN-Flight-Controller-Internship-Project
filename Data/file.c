#include <stdio.h>

int main(){
    char nama[14], nama2[3], nama3[3];
	FILE *PID_A=fopen("in.txt","r");
    fscanf(PID_A,"%s\n", &nama);fflush(stdin);   
    // %[^#] artinya kita menyimpan bagian dari string dalam file sampai tanda #. 
    // Kita tidak menggunnakan %s karena nama mengandung spasi
    printf("%s\n", nama);
        
		
	fclose(PID_A);
	
        return 0;
}