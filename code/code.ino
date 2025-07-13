#define ENA_FO 125
#define ENB_FO 125
#define ENA_OR 75
#define ENB_OR 125
#define ENA_CR 0
#define ENB_CR 125
#define ENA_OL 125
#define ENB_OL 75
#define ENA_CL 125
#define ENB_CL 0
#define nnhl 7
#define lr 0.25
#define tn 0.01
//Δηλώνουμε ως σταθερές τους ακροδέκτες του Motor Driver και τις αρχικοποιούμε με τους αντίστοιχους ακροδέκτες του Teensy 3.2.
const int ENA=5;
const int IN1=1;
const int IN2=2;
const int IN3=3;
const int IN4=4;
const int ENB=6;
//Δηλώνουμε ως σταθερές τους ακροδέκτες των αισθητήρων απόστασης και τις αρχικοποιούμε με τους αντίστοιχους ακροδέκτες του Teensy 3.2.
const int LSEP=22;
const int LSTP=21;
const int MSEP=20;
const int MSTP=19;
const int RSEP=18;
const int RSTP=17;
//Δηλώνουμε το πρότυπο μιας δομής τύπου distances.
struct distances
{
  int d1;
  int d2;
  int d3;
};
//Δηλώνουμε το πρότυπο μιας δομής τύπου networkoutputs.
struct networkoutputs
{
  int o1;
  int o2;
};
//Δηλώνουμε τη συνάρτηση που είναι υπεύθυνη για το διάβασμα της απόστασης μεταξύ του εμποδίου και του κάθε αισθητήρα απόστασης.
struct distances read_sensors_data();
//Δηλώνουμε τη συνάρτηση που είναι υπεύθυνη για τον υπολογισμό των δυο εξόδων του τεχνητού νευρωνικού δικτύου ανάλογα με την απόσταση του εμποδίου από τον κάθε αισθητήρα απόστασης.
struct networkoutputs compute_neural_network_outputs(struct distances getdis);
//Δηλώνουμε τη συνάρτηση που είναι υπεύθυνη για την κίνηση του ρομπότ ανάλογα με τις τιμές των δυο εξόδων του τεχνητού νευρωνικού δικτύου.
void move_robot(struct networkoutputs getno);
float wh[4][nnhl],wo[nnhl+1][2];
int i,j,mintid,maxtid;

void setup()
{
  struct distances td;
  struct networkoutputs tno;
  float afh[nnhl+1];
  float afo[2];
  float sum1=0,uswboduwo,uafoduswbo,uEtduafo,uEtduwo,uswbhduwh,uafhduswbh,uEtduafh,uEtduwh,pwo[nnhl+1][2],v1,v2,tr1,tr2;
  float tidan[161][4],todan[161][2];
  float mse;
  int k=0;
  //Δηλώνουμε τον πίνακα που περιέχει τις τιμές των εισόδων του τεχνητού νευρωνικού δικτύου.
  //Αριστερός Αισθητήρας , Κεντρικός Αισθητήρας , Δεξιός Αισθητήρας.
  int tid[161][3]={{167,19,126},
                  {169,15,130},
                  {169,18,124},
                  {212,18,128},
                  {211,15,131},
                  {169,15,126},
                  {212,19,123},
                  {168,16,140},
                  {167,20,122},
                  {168,19,124},
                  {168,16,128},
                  {169,17,125},
                  {168,20,122},
                  {168,18,125},
                  {212,16,131},
                  {168,17,126},
                  {167,19,141},
                  {169,20,123},
                  {168,14,142},
                  {212,18,123},
                  {169,14,130},
                  {168,15,129},
                  {167,20,140},
                  {14,105,182},
                  {20,108,182},
                  {14,129,182},
                  {16,123,181},
                  {14,130,181},
                  {15,125,181},
                  {17,122,181},
                  {14,126,181},
                  {16,127,183},
                  {16,104,181},
                  {18,121,180},
                  {19,105,180},
                  {20,123,183},
                  {15,129,182},
                  {15,107,183},
                  {17,105,180},
                  {18,125,181},
                  {15,130,182},
                  {19,127,181},
                  {16,107,183},
                  {20,119,181},
                  {15,104,181},
                  {20,120,180},
                  {115,106,19},
                  {119,107,19},
                  {116,106,19},
                  {116,107,16},
                  {116,107,20},
                  {118,107,17},
                  {121,105,15},
                  {122,107,16},
                  {121,105,19},
                  {120,107,15},
                  {115,107,16},
                  {116,107,14},
                  {122,107,15},
                  {120,107,16},
                  {119,105,15},
                  {118,106,19},
                  {115,108,18},
                  {123,105,18},
                  {121,107,16},
                  {115,107,20},
                  {120,105,17},
                  {118,106,18},
                  {119,108,17},
                  {105,16,18},
                  {122,17,18},
                  {138,17,18},
                  {136,17,18},
                  {122,14,15},
                  {107,14,15},
                  {109,15,15},
                  {122,15,15},
                  {106,20,19},
                  {123,16,16},
                  {138,16,16},
                  {107,15,18},
                  {129,19,20},
                  {132,18,20},
                  {106,16,17},
                  {110,18,19},
                  {148,17,18},
                  {105,20,20},
                  {176,16,18},
                  {120,15,16},
                  {108,16,16},
                  {125,15,16},
                  {140,16,16},
                  {14,14,105},
                  {18,19,106},
                  {15,16,118},
                  {14,15,112},
                  {16,15,119},
                  {17,17,108},
                  {15,15,106},
                  {15,15,110},
                  {20,20,106},
                  {19,19,112},
                  {16,16,113},
                  {16,16,106},
                  {16,17,118},
                  {17,18,105},
                  {19,15,193},
                  {16,17,112},
                  {18,19,110},
                  {20,20,110},
                  {19,15,192},
                  {14,14,127},
                  {19,19,106},
                  {17,18,113},
                  {14,15,125},
                  {16,14,17},
                  {17,14,17},
                  {19,14,18},
                  {17,15,19},
                  {18,14,20},
                  {19,15,18},
                  {19,16,20},
                  {19,15,20},
                  {16,14,18},
                  {17,14,18},
                  {17,14,20},
                  {18,15,20},
                  {18,14,19},
                  {17,15,18},
                  {18,14,17},
                  {18,15,19},
                  {19,15,19},
                  {16,14,19},
                  {19,14,20},
                  {18,16,20},
                  {19,16,19},
                  {17,14,19},
                  {18,14,18},
                  {161,155,162},
                  {166,161,166},
                  {161,167,170},
                  {175,166,170},
                  {164,159,165},
                  {162,158,125},
                  {165,160,165},
                  {174,168,171},
                  {165,162,167},
                  {165,161,164},
                  {167,161,165},
                  {161,157,163},
                  {165,159,166},
                  {164,159,162},
                  {167,161,167},
                  {161,159,119},
                  {168,162,167},
                  {168,162,165},
                  {168,163,165},
                  {168,161,165},
                  {164,158,164},
                  {194,100,177},
                  {166,159,162}};
  //Δηλώνουμε τον πίνακα που περιέχει τις τιμές των στόχων του τεχνητού νευρωνικού δικτύου.
  //Δεξιός Τροχός , Αριστερός Τροχός.
  int tod[161][2]={{ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OR,ENB_OR},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_OL,ENB_OL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CL,ENB_CL},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_CR,ENB_CR},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO},
                  {ENA_FO,ENB_FO}};
  //Δηλώνουμε τους ακροδέκτες του Teensy 3.2 που συνδέονται στους ακροδέκτες του Motor Driver ως εξόδους.
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  //Δηλώνουμε τους ακροδέκτες του Teensy 3.2 που συνδέονται στους ακροδέκτες Trig των αισθητήρων απόστασης ως εξόδους.
  pinMode(LSTP,OUTPUT);
  pinMode(MSTP,OUTPUT);
  pinMode(RSTP,OUTPUT);
  //Δηλώνουμε τους ακροδέκτες του Teensy 3.2 που συνδέονται στους ακροδέκτες Echo των αισθητήρων απόστασης ως εισόδους.
  pinMode(LSEP,INPUT);
  pinMode(MSEP,INPUT);
  pinMode(RSEP,INPUT);
  randomSeed(micros());
  //Αρχικοποιούμε τον πίνακα που περιέχει τα βάρη του κρυφού επιπέδου με τυχαίες τιμές μεταξύ 0 και 1.
  for(i=0;i<4;i++)
  {
    for(j=0;j<nnhl;j++)
    {
      v1=random(11);
      v2=random(11);
      wh[i][j]=(v1/10.0)*(v2/10.0);
    }
  }
  //Αρχικοποιούμε τον πίνακα που περιέχει τα βάρη του επιπέδου εξόδου με τυχαίες τιμές μεταξύ 0 και 1.
  for(i=0;i<(nnhl+1);i++)
  {
    for(j=0;j<2;j++)
    {
      v1=random(11);
      v2=random(11);
      wo[i][j]=(v1/10.0)*(v2/10.0);
    }
  }
  //Βρίσκουμε την ελάχιστη και τη μέγιστη τιμή του πίνακα που περιέχει τις τιμές των εισόδων του τεχνητού νευρωνικού δικτύου.
  mintid=400;
  maxtid=2;
  for(i=0;i<161;i++)
  {
    for(j=0;j<3;j++)
    {
      if(tid[i][j]>=maxtid)
      {
        maxtid=tid[i][j];
      }
      if(tid[i][j]<=mintid)
      {
        mintid=tid[i][j];
      }
    }
  }
  //Κανονικοποιούμε τις τιμές των εισόδων ώστε να βρίσκονται μεταξύ 0 και 1 και σε κάθε στοιχείο της τελευταίας στήλης του καινούργιου πίνακα εκχωρούμε τη σταθερή είσοδο.
  for(i=0;i<161;i++)
  {
    for(j=0;j<4;j++)
    {
      if(j!=3)
      {
        tidan[i][j]=((float)(tid[i][j]-mintid))/((float)(maxtid-mintid));
      }
      else
      {
        tidan[i][j]=1.0;
      }
    }
  }
  //Κανονικοποιούμε τις τιμές των στόχων ώστε να βρίσκονται μεταξύ 0 και 1.
  for(i=0;i<161;i++)
  {
    for(j=0;j<2;j++)
    {
      todan[i][j]=((float)(tod[i][j]))/125.0; 
    }
  }
  do
  {
    while(k<161)
    {
      //Εκτελούμε Forward Pass και υπολογίζουμε την έξοδο του κάθε νευρώνα του κρυφού επιπέδου και την έξοδο του κάθε νευρώνα του επιπέδου εξόδου. Στο στοιχείο της τελευταίας στήλης του μονοδιάστατου πίνακα που περιέχει τις τιμές των εξόδων του κάθε νευρώνα του κρυφού επιπέδου εκχωρούμε τη σταθερή είσοδο.
      for(i=0;i<(nnhl+1);i++)
      {
        if(i!=nnhl)
        {
          for(j=0;j<4;j++)
          {
            sum1=sum1+(tidan[k][j]*wh[j][i]);
          }
          afh[i]=1.0/(1.0+(exp(-sum1)));
          sum1=0;
        }
        else
        {
          afh[i]=1.0;
        }
      }
      for(i=0;i<2;i++)
      {
        for(j=0;j<(nnhl+1);j++)
        {
          sum1=sum1+(afh[j]*wo[j][i]);
        }
        afo[i]=1.0/(1.0+(exp(-sum1)));
        sum1=0;
      }
      //Εκτελούμε Backward Pass και διορθώνουμε τα βάρη του επιπέδου εξόδου και τα βάρη του κρυφού επιπέδου.
      for(i=0;i<(nnhl+1);i++)
      {
        for(j=0;j<2;j++)
        {
          pwo[i][j]=wo[i][j];
        }
      }
      for(i=0;i<2;i++)
      {
        for(j=0;j<(nnhl+1);j++)
        {
          uswboduwo=afh[j];
          uafoduswbo=afo[i]*(1.0-afo[i]);
          uEtduafo=afo[i]-todan[k][i];
          uEtduwo=uEtduafo*uafoduswbo*uswboduwo;
          wo[j][i]=wo[j][i]-(lr*uEtduwo);
        }
      }
      for(i=0;i<nnhl;i++)
      {
        for(j=0;j<4;j++)
        {
          uswbhduwh=tidan[k][j];
          uafhduswbh=afh[i]*(1.0-afh[i]);
          uEtduafh=((afo[0]-todan[k][0])*(afo[0]*(1.0-afo[0]))*pwo[i][0])+((afo[1]-todan[k][1])*(afo[1]*(1.0-afo[1]))*pwo[i][1]);
          uEtduwh=uEtduafh*uafhduswbh*uswbhduwh;
          wh[j][i]=wh[j][i]-(lr*uEtduwh);
        }
      }
      //Παίρνουμε το επόμενο πρότυπο.
      k=k+1;  
    }
    //Υπολογίζουμε το μέσο τετραγωνικό σφάλμα των εξόδων του τεχνητού νευρωνικού δικτύου για όλα τα πρότυπα.
    for(k=0;k<161;k++)
    {
      td.d1=tid[k][0];
      td.d2=tid[k][1];
      td.d3=tid[k][2];
      tno=compute_neural_network_outputs(td);
      tr1=(float)pow(todan[k][0]-(((float)tno.o1)/125.0),2.0);
      tr2=(float)pow(todan[k][1]-(((float)tno.o2)/125.0),2.0);
      sum1=sum1+tr1+tr2;
    }
    mse=sum1/322.0;
    sum1=0;
    k=0;
    //Ελέγχουμε αν το μέσο τετραγωνικό σφάλμα είναι μεγαλύτερο από το κατώφλι τερματισμού. Αν είναι μεγαλύτερο ο αλγόριθμος εκπαίδευσης ξεκινάει από την αρχή ενώ αν είναι μικρότερο ή ίσο ο αλγόριθμος εκπαίδευσης σταματάει.
  }while(mse>tn);
}

void loop()
{
  struct distances ld;
  struct networkoutputs lno;
  ld=read_sensors_data();
  lno=compute_neural_network_outputs(ld);
  move_robot(lno); 
}

//Ορίζουμε τη συνάρτηση που διαβάζει την απόσταση του εμποδίου από τον κάθε αισθητήρα απόστασης.
struct distances read_sensors_data()
{
  struct distances retdis;
  float LSDU,MSDU,RSDU;
  int LSDI,MSDI,RSDI,nLSDI,nMSDI,nRSDI;
  digitalWrite(LSTP,LOW);
  digitalWrite(MSTP,LOW);
  digitalWrite(RSTP,LOW);
  delayMicroseconds(2);
  //Διαβάζουμε την απόσταση μεταξύ του εμποδίου και του αριστερού αισθητήρα.
  digitalWrite(LSTP,HIGH);
  delayMicroseconds(10);
  digitalWrite(LSTP,LOW);
  LSDU=pulseIn(LSEP,HIGH);
  LSDI=(int)round((LSDU/2.0)*0.0343);
  //Διαβάζουμε την απόσταση μεταξύ του εμποδίου και του κεντρικού αισθητήρα.
  digitalWrite(MSTP,HIGH);
  delayMicroseconds(10);
  digitalWrite(MSTP,LOW);
  MSDU=pulseIn(MSEP,HIGH);
  MSDI=(int)round((MSDU/2.0)*0.0343);
  //Διαβάζουμε την απόσταση μεταξύ του εμποδίου και του δεξιού αισθητήρα.
  digitalWrite(RSTP,HIGH);
  delayMicroseconds(10);
  digitalWrite(RSTP,LOW);
  RSDU=pulseIn(RSEP,HIGH);
  RSDI=(int)round((RSDU/2.0)*0.0343);
  //Αλλάζουμε τις τιμές των αποστάσεων που διαβάσαμε ώστε να μην είναι μεγαλύτερες από τη μέγιστη τιμή του πίνακα που περιέχει τις τιμές των εισόδων και μικρότερες από την ελάχιστη τιμή του ίδιου πίνακα. 
  nLSDI=constrain(LSDI,mintid,maxtid);
  nMSDI=constrain(MSDI,mintid,maxtid);
  nRSDI=constrain(RSDI,mintid,maxtid);
  //Επιστρέφουμε τις τιμές των αποστάσεων που διαβάσαμε.
  retdis.d1=nLSDI;
  retdis.d2=nMSDI;
  retdis.d3=nRSDI;
  return retdis;
}

//Ορίζουμε τη συνάρτηση που υπολογίζει τις δυο εξόδους του τεχνητού νευρωνικού δικτύου.
struct networkoutputs compute_neural_network_outputs(struct distances getdis)
{
  struct networkoutputs retno;
  float sdman[4],hl[nnhl+1];
  float ol[2];
  float sum2=0;
  int olad[2];
  //Κανονικοποιούμε τα δεδομένα που διαβάσαμε από τους αισθητήρες απόστασης ώστε να βρίσκονται μεταξύ 0 και 1.
  sdman[0]=((float)(getdis.d1-mintid))/((float)(maxtid-mintid));
  sdman[1]=((float)(getdis.d2-mintid))/((float)(maxtid-mintid));
  sdman[2]=((float)(getdis.d3-mintid))/((float)(maxtid-mintid));
  sdman[3]=1.0;
  //Υπολογίζουμε την έξοδο του κάθε νευρώνα του κρυφού επιπέδου. Στο στοιχείο της τελευταίας στήλης του μονοδιάστατου πίνακα που περιέχει τις τιμές των εξόδων του κάθε νευρώνα του κρυφού επιπέδου εκχωρούμε τη σταθερή είσοδο.
   for(j=0;j<(nnhl+1);j++)
   {
    if(j!=nnhl)
    {
      for(i=0;i<4;i++)
      {
        sum2=sum2+(sdman[i]*wh[i][j]);
      }
      hl[j]=1.0/(1.0+(exp(-sum2)));
      sum2=0;
    }
    else
    {
      hl[j]=1.0;
    }
  }
  //Υπολογίζουμε τις δυο εξόδους του τεχνητού νευρωνικού δικτύου.
  for(i=0;i<2;i++)
  {
    for(j=0;j<(nnhl+1);j++)
    {
      sum2=sum2+(hl[j]*wo[j][i]);
    }
    ol[i]=1.0/(1.0+(exp(-sum2)));
    sum2=0;
  }
  //Αποκανονικοποιούμε τις δυο εξόδους του τεχνητού νευρωνικού δικτύου ώστε να βρίσκονται μεταξύ 0 και 125.
  for(i=0;i<2;i++)
  {
    olad[i]=(int)(ol[i]*125.0);
  }
  //Επιστρέφουμε τις τιμές των δυο εξόδων του τεχνητού νευρωνικού δικτύου.
  retno.o1=olad[0];
  retno.o2=olad[1];
  return retno;
}

//Ορίζουμε τη συνάρτηση για την κίνηση του ρομπότ.
void move_robot(struct networkoutputs getno)
{
  //Μεταβάλλουμε την ταχύτητα του ενός τροχού ή και των δυο τροχών ταυτόχρονα σύμφωνα με τις δυο εξόδους του τεχνητού νευρωνικού δικτύου ώστε το ρομπότ να κινηθεί προς τη σωστή κατεύθυνση προκειμένου να αποφύγει το εμπόδιο. 
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,getno.o1);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENB,getno.o2);
}
