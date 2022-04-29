//***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                   
// - Karthik Nadig, USA                                                  
// Please report bugs to: karthiknadig@gmail.com                         
//***********************************************************************
#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

//***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                   
// - Karthik Nadig, USA                                                  
// Please report bugs to:                                                
// https://github.com/karthiknadig/ArduinoFIS/issues                     
// If you don't have a GitHub account mail to karthiknadig@gmail.com     
//***********************************************************************


// Number of inputs to the fuzzy inference system
const int fis_gcI = 2;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 36;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];


//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 6, 6 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 6 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0.193563068920676, 5.92356306892068, 10.9435630689207 };
FIS_TYPE fis_gMFI0Coeff2[] = { 4.85636363636363, 13.7863636363636, 24.1863636363636 };
FIS_TYPE fis_gMFI0Coeff3[] = { 14.625, 28.9, 47.1 };
FIS_TYPE fis_gMFI0Coeff4[] = { 27.2, 44.2, 59.0113636363636 };
FIS_TYPE fis_gMFI0Coeff5[] = { 49.9, 80.2, 134.897727272727 };
FIS_TYPE fis_gMFI0Coeff6[] = { 62.9, 100, 142 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5, fis_gMFI0Coeff6 };
FIS_TYPE fis_gMFI1Coeff1[] = { -0.03467, 0.02263, 0.04953 };
FIS_TYPE fis_gMFI1Coeff2[] = { 0.0114, 0.0843823146944082, 0.169 };
FIS_TYPE fis_gMFI1Coeff3[] = { 0.0456, 0.20224967490247, 0.357 };
FIS_TYPE fis_gMFI1Coeff4[] = { 0.186284785435631, 0.414284785435631, 0.630284785435631 };
FIS_TYPE fis_gMFI1Coeff5[] = { 0.377, 0.797, 1.23358907672302 };
FIS_TYPE fis_gMFI1Coeff6[] = { 0.774, 1.17465539661899, 2.2 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4, fis_gMFI1Coeff5, fis_gMFI1Coeff6 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 20, 648.8, 1013 };
FIS_TYPE fis_gMFO0Coeff2[] = { 772.3, 1091, 1322 };
FIS_TYPE fis_gMFO0Coeff3[] = { 1076, 1612, 2068 };
FIS_TYPE fis_gMFO0Coeff4[] = { 1607, 2137, 2644 };
FIS_TYPE fis_gMFO0Coeff5[] = { 2484, 2841, 3210 };
FIS_TYPE fis_gMFO0Coeff6[] = { 2755, 3181, 3706 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5, fis_gMFO0Coeff6 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0, 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0, 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1 };
int fis_gRI1[] = { 1, 2 };
int fis_gRI2[] = { 1, 3 };
int fis_gRI3[] = { 1, 4 };
int fis_gRI4[] = { 1, 5 };
int fis_gRI5[] = { 1, 6 };
int fis_gRI6[] = { 2, 1 };
int fis_gRI7[] = { 3, 1 };
int fis_gRI8[] = { 4, 1 };
int fis_gRI9[] = { 5, 1 };
int fis_gRI10[] = { 6, 1 };
int fis_gRI11[] = { 2, 2 };
int fis_gRI12[] = { 2, 3 };
int fis_gRI13[] = { 2, 4 };
int fis_gRI14[] = { 2, 5 };
int fis_gRI15[] = { 2, 6 };
int fis_gRI16[] = { 3, 2 };
int fis_gRI17[] = { 4, 2 };
int fis_gRI18[] = { 5, 2 };
int fis_gRI19[] = { 6, 2 };
int fis_gRI20[] = { 3, 3 };
int fis_gRI21[] = { 3, 4 };
int fis_gRI22[] = { 3, 5 };
int fis_gRI23[] = { 3, 6 };
int fis_gRI24[] = { 4, 3 };
int fis_gRI25[] = { 5, 3 };
int fis_gRI26[] = { 6, 3 };
int fis_gRI27[] = { 4, 4 };
int fis_gRI28[] = { 4, 5 };
int fis_gRI29[] = { 4, 6 };
int fis_gRI30[] = { 5, 4 };
int fis_gRI31[] = { 6, 4 };
int fis_gRI32[] = { 5, 5 };
int fis_gRI33[] = { 5, 6 };
int fis_gRI34[] = { 6, 5 };
int fis_gRI35[] = { 6, 6 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26, fis_gRI27, fis_gRI28, fis_gRI29, fis_gRI30, fis_gRI31, fis_gRI32, fis_gRI33, fis_gRI34, fis_gRI35 };

// Rule Outputs
int fis_gRO0[] = { 6 };
int fis_gRO1[] = { 5 };
int fis_gRO2[] = { 4 };
int fis_gRO3[] = { 3 };
int fis_gRO4[] = { 2 };
int fis_gRO5[] = { 1 };
int fis_gRO6[] = { 2 };
int fis_gRO7[] = { 3 };
int fis_gRO8[] = { 4 };
int fis_gRO9[] = { 5 };
int fis_gRO10[] = { 6 };
int fis_gRO11[] = { 5 };
int fis_gRO12[] = { 4 };
int fis_gRO13[] = { 3 };
int fis_gRO14[] = { 2 };
int fis_gRO15[] = { 1 };
int fis_gRO16[] = { 3 };
int fis_gRO17[] = { 4 };
int fis_gRO18[] = { 5 };
int fis_gRO19[] = { 6 };
int fis_gRO20[] = { 4 };
int fis_gRO21[] = { 3 };
int fis_gRO22[] = { 2 };
int fis_gRO23[] = { 1 };
int fis_gRO24[] = { 4 };
int fis_gRO25[] = { 5 };
int fis_gRO26[] = { 6 };
int fis_gRO27[] = { 3 };
int fis_gRO28[] = { 2 };
int fis_gRO29[] = { 1 };
int fis_gRO30[] = { 5 };
int fis_gRO31[] = { 6 };
int fis_gRO32[] = { 2 };
int fis_gRO33[] = { 1 };
int fis_gRO34[] = { 6 };
int fis_gRO35[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26, fis_gRO27, fis_gRO28, fis_gRO29, fis_gRO30, fis_gRO31, fis_gRO32, fis_gRO33, fis_gRO34, fis_gRO35 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 5, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 145, 2.06 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 550 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 3200 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
