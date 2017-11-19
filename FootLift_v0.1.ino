
/*
  Copyright (c) 2013 Simon Howard
  Permission to use, copy, modify, and/or distribute this software
  for any purpose with or without fee is hereby granted, provided
  that the above copyright notice and this permission notice appear
  in all copies.
  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
  WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
  AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
  CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
  LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
  CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

// Yoctolisp: a miniature lisp interpreter written in a weekend.

/*
    Author: Tim Barrett
    Date: 19th Sept 2017
    Purpose: Arduino code to send and receive 20 characters at a time
      to from a source of bytes. I an using nRF UART v2.0 android app
      sensitivusmk/Android-nRF-UART forked from NordicSemiconductor/Android-nRF-UART
       this is stage one of getting a minimal lisp interpreter workng over BLE

       - minimal BLE uart lisp starting
       - (setdate yy mm dd)
       - (settime hh mm ss)
       - (memory)
       - (dwrite pin value)

        bug:  (define a 7)
              (define b 6)
              (add a b) --> 'Invalid function cal' over BLE
        bug: when serial gets (8316 1880 6392) app gets ')'
        bug: when a call is made to an undefined thing eval loop hangs
              undefined variable Invalid function call

*/

// porting it to Arduino 101 is taking considerably longer than a weekend

/*
  // TODO test
  // add sub mul div lt eq and or not car cdr cons read eval print
  // if quote cond lambda define let begin

  ;; make all safe
  (dw 4 0)              ; digital write pin 4 to LOW
  (dw 5 0)              ; digital write pin 5 to LOW
  (dw 6 0)              ; digital write pin 6 to LOW
  (dw 7 0)              ; digital write pin 7 to LOW

  ;; set wipers to zero
  (i2c-write 44 0)      ; select digital potentiometer on channel 0
  (i2c-write 44 32)     ; set it to halfway on
  (i2c-write 44 0)      ; select digital potentiometer on channel 0
  (i2c-write 44 32)     ; set it to halfway on

  'ylisp' enabled over ble - input limited to 20 characters - 107944 bytes
  added RTC - 124368 bytes (16,424b) // seems excsessive
  added memory - 125088 bytes (720b)
  aded i2c write - 128824 bytes (3,736b)
  fixed null pointer problem - 130220 bytes

  maximum is 155648. (6904 1836 5024) free stack heap


*/

#include <CurieBLE.h> // needed for BLE uart 
#include <MemoryFree.h> // needed for (memory) --> (free stack . heap)
#ifndef trbtime
//#include <CurieTime.h> // needed for (datetime hh mm ss dd mm yy)
#endif
#include <CurieIMU.h>
#include <Wire.h> // needed for (i2c address int1 int2)

#include <SPI.h>
#include <SD.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <ctype.h>
#include <assert.h>

//#define GC_DEBUG

#ifdef GC_DEBUG  // How many objects to allocate before triggering a GC?
#define GC_ALLOC_TRIGGER 1
#else
#define GC_ALLOC_TRIGGER 10
#endif

#define LEX_DEBUG(x) //Serial.println(x);
#define PARSE_DEBUG(x)  // Serial.print(x);
#define EVAL_DEBUG(x) //Serial.print(x);
#define LOOP_DEBUG(x) //Serial.print(x);

/* BLE */

const char* localName = "yLisp";

static const int BLE_MAX_LENGTH = 20;
char RXBuffer[(BLE_MAX_LENGTH) + 1]; // just an input buffer
char LPE_code[(BLE_MAX_LENGTH * 2) + 2]; // capable of storing two full rx's +' '&'\0'
int number_of_closed_parentheses = 0;
int number_of_opened_parentheses = 0;
boolean code_flagged_as_failing_to_be_processed_for_eval = false;
//char* preamble = "6E40000";
//char* post = "-B5A3-F393-E0A9-E50E24DCCA9E";
//BLEService uartService = BLEService(preamble+"1"+post);
BLEService uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEDescriptor uartNameDescriptor = BLEDescriptor("2901", localName);

BLECharacteristic rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, BLE_MAX_LENGTH);
BLEDescriptor rxNameDescriptor = BLEDescriptor("2901", "RX - (Write)");
BLECharacteristic txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLEIndicate, BLE_MAX_LENGTH);
BLEDescriptor txNameDescriptor = BLEDescriptor("2901", "TX - (Indicate)");

void rxCharacteristicWritten(BLECentral & central, BLECharacteristic & characteristic);
boolean outputMemoryOnce = false;

/* TRB yocto additions */

char outputBuffer[BLE_MAX_LENGTH];
int filledTo = 0;

#define mirrorREPLToSerial 1 // if non zero everything sent to BLE tx gets sent to serial too

/* end TRB yocto addition */

/* end BLE definitions */

// ======================== leg stim =====================

#define Addr 0x2C   // AD5252 I2C address is 0x2C(44)

//Following variables declared before void setup to make them global
//D0 & D1 reserved for serial Bluetooth if needed

int Blue_LED = 2;   //Blue LED
int Green_LED = 3;  //Green LED
int Red_LED = 8;  //Red LED
int Ch1a = 4;     //Switches on positive side of Ch1 EMS to electrodes
int Ch1b = 5;     //Switches on negative side of Ch1 EMS to electrodes
int Ch2a = 7;     //Switches on positive side of Ch2 EMS to electrodes
int Ch2b = 6;     //Switches on negativeside of Ch2 EMS to electrodes
int BNO055_Int = 9;  //Onboard sensor interrupt input
const int chipSelect = 10; // 11,12,13 also implicated in SPI 
//D10,11,12,13 for SPI interface to SD card (D10 is CS)

int Remote_LED = A0;  //Green LED on manual remote
int Remote_Pot = A1;  //Potentiometer on manual remote 0-1023
int Remote_Sw1 = A2;  //Switch 1 on Remote (active low)
int Remote_Sw2 = A3; //Switch 3 on Remote (active low)

//A4 and A5 reserved for SDA/SCL I2C interface

byte chan1 = 1;                              //Channel 1
byte chan2 = 2;                              //Channel 2
byte pot_val;                                //Temp value to hold value of pot
bool sw_val1;                                //Temp value of switch 1
bool sw_val2;                                //Temp value of switch 2
byte load;                                   //Temp value of load on AD5252

// for step detection
long lastStepCount = 0;

typedef enum {
  YLISP_CELL, YLISP_STRING, YLISP_NUMBER, YLISP_BOOLEAN, YLISP_SYMBOL,
  YLISP_CONTEXT, YLISP_FUNCTION, YLISP_BUILTIN
} YLispValueType;

enum {
  KWD_IF, KWD_COND, KWD_QUOTE, KWD_LAMBDA, KWD_LET, KWD_DEFINE,
  KWD_BEGIN, NUM_KEYWORDS
};

typedef struct _YLispValue *(*YLispBuiltin)(struct _YLispValue *args);

void BLEAppend(const char *text);

typedef struct _YLispValue {
  unsigned int marked : 1;
  unsigned int type : 31;
  union {
    unsigned int i;
    char *s;
    struct {
      struct _YLispValue *cdr, *car;
    } cell;
    struct _YLispValue *symname;
    struct {
      struct _YLispValue *parent, *vars;
    } context;
    struct {
      struct _YLispValue *context, *code;
    } func;
    YLispBuiltin builtin;
  } v;
  struct _YLispValue *next;
} YLispValue;

#define CAR(val) ((val)->v.cell.car)
#define CDR(val) ((val)->v.cell.cdr)

typedef enum {
  TOKEN_OPEN_PAREN, TOKEN_CLOSE_PAREN, TOKEN_LITERAL, TOKEN_QUOTE,
  TOKEN_ERROR, TOKEN_EOF, TOKEN_PERIOD
} YLispToken;

typedef struct {
  char *buffer;
  unsigned int position;
  YLispValue *value;
} YLispLexer;

typedef struct _GCPinnedVariable {
  YLispValue **variable;
  struct _GCPinnedVariable *next;
} GCPinnedVariable;

static const char *keyword_names[] = {
  "if", "cond", "quote", "fn", "let", "def", "begin"
};

static YLispValue *values = NULL;  // All values linked list
static unsigned int values_alloc_count = 0;  // Allocated since last GC
static GCPinnedVariable *pinned_vars = NULL;

static YLispValue *keywords[NUM_KEYWORDS];

// Symbol table; dynamically resizing.
static YLispValue **symbols = NULL;
static unsigned int num_symbols = 0;

static YLispValue *root_context;

// A deferred call, passed up the stack (only one ever exists)
static YLispValue deferred_call;

static YLispValue *ylisp_value(YLispValueType type)
{
  YLispValue *result = ((YLispValue*)(calloc(1, sizeof(YLispValue))));
  assert(result != NULL);
  result->type = type;
  result->next = values; values = result;
  ++values_alloc_count;
  return result;
}

static YLispValue *ylisp_number(YLispValueType type, unsigned int val)
{
  YLispValue *result = ylisp_value(type);
  result->v.i = val;
  return result;
}

static YLispValue *ylisp_cons(YLispValue *car, YLispValue *cdr)
{
  YLispValue *result = ylisp_value(YLISP_CELL);
  CAR(result) = car; CDR(result) = cdr;
  return result;
}


// ===============================================================================
// Printing
// ===============================================================================

void ylisp_print(YLispValue *value);

static void print_list(YLispValue *value, char *inner)
{
  // strings of char are \0 terminated to automate length calculation.

  //printf("(%s", inner); 3 lines
  static  char openParen[2] = {'(', '\0'};
  BLEAppend(openParen);

  BLEAppend((char*)inner);

  while (value != NULL) {
    if (value->type != YLISP_CELL) {
      // printf(". "); 2 lines
      static  char dotSpace[3] = {'.', ' ', '\0'};
      BLEAppend(dotSpace);
      ylisp_print(value);
      break;
    }
    ylisp_print(CAR(value));
    value = CDR(value);
    if (value != NULL) {
      //printf(" "); 2 lines
      static  char space[2] = {' ', '\0'};
      BLEAppend(space);

    }
  }
  // printf(")"); 2 lines
  static  char closeParen[2] = {')', '\0'};
  BLEAppend(closeParen);
}

void ylisp_print(YLispValue *value)
{
  if (value == NULL) {
    static  char openClose[4] = {'(', ')', '\n', '\0'};
    BLEAppend(openClose);
  } else switch (value->type) {
      case YLISP_CELL:
        print_list(value, "");
        break;
      case YLISP_STRING:
        static  char quotationMarks[2] = {'\"', '\0'};
        BLEAppend(quotationMarks);
        BLEAppend(( char*)value->v.s);
        BLEAppend(quotationMarks);
        break;
      case YLISP_NUMBER:
        static char numBuffer[10];
        Itoa(value->v.i, numBuffer, 10);
        BLEAppend(numBuffer);
        break;
      case YLISP_BOOLEAN:
        printf("#%c", value->v.i ? 't' : 'f');
        break;
      case YLISP_SYMBOL:
        BLEAppend(( char*)value->v.symname->v.s);
        break;
      case YLISP_CONTEXT:
        static  char context[10] = {'<', 'c', 'o', 'n', 't', 'e', 'x', 't', '>', '\0'};
        BLEAppend(context);
        break;
      case YLISP_FUNCTION:
        print_list(value->v.func.code, "lambda ");
        break;
      case YLISP_BUILTIN:
        static  char builtIn[10] = {'<', 'b', 'u', 'i', 'l', 't', 'i', 'n', '>', '\0'};
        BLEAppend(builtIn);
        break;
    }
}

/* The Itoa code is in the public domain */
char* Itoa(int value,  char* str, int radix) {
  static  char dig[] =
    "0123456789"
    "abcdefghijklmnopqrstuvwxyz";
  int n = 0, neg = 0;
  unsigned int v;
  char* p, *q;
  char c;
  if (radix == 10 && value < 0) {
    value = -value;
    neg = 1;
  }
  v = value;
  do {
    str[n++] = dig[v % radix];
    v /= radix;
  } while (v);
  if (neg)
    str[n++] = '-';
  str[n] = '\0';
  for (p = str, q = p + (n - 1); p < q; ++p, --q)
    c = *p, *p = *q, *q = c;
  return str;
}
static int ylisp_equal(YLispValue *v1, YLispValue *v2)
{
  if (v1 == NULL || v2 == NULL) {
    return v1 == v2;
  }
  if (v1->type != v2->type) {
    return 0;
  }
  switch (v1->type) {
    case YLISP_STRING:
      return !strcmp(v1->v.s, v2->v.s);
    case YLISP_NUMBER:
    case YLISP_BOOLEAN:
      return v1->v.i == v2->v.i;
    default:
      return v1 == v2;
  }
}

static int ylisp_lt(YLispValue *v1, YLispValue *v2)
{
  if (v1 == NULL || v2 == NULL || v1->type != v2->type) {
    return 0;
  }
  switch (v1->type) {
    case YLISP_STRING:
      return strcmp(v1->v.s, v2->v.s) < 0;
    case YLISP_NUMBER:
    case YLISP_BOOLEAN:
      return v1->v.i < v2->v.i;
    default:
      return 0;
  }
}

// ===============================================================================
// Garbage Collection
// ===============================================================================

static void mark_value(YLispValue *value)
{
  if (value == NULL || value->marked) {
    return;
  }
  value->marked = 1;

  switch (value->type) {
    case YLISP_CELL:
      mark_value(value->v.cell.cdr);
      mark_value(value->v.cell.car);
      break;
    case YLISP_SYMBOL:
      mark_value(value->v.symname);
      break;
    case YLISP_CONTEXT:
      mark_value(value->v.context.parent);
      mark_value(value->v.context.vars);
      break;
    case YLISP_FUNCTION:
      mark_value(value->v.func.context);
      mark_value(value->v.func.code);
      break;
    default: break;
  }
}

static void mark_roots(void)
{
  GCPinnedVariable *pin;
  unsigned int i;
  for (i = 0; i < num_symbols; ++i) {
    mark_value(symbols[i]);
  }
  for (pin = pinned_vars; pin != NULL; pin = pin->next) {
    mark_value(*pin->variable);
  }
  mark_value(root_context);
}

static void free_value(YLispValue *value)
{
  if (value->type == YLISP_STRING) {
    free(value->v.s);
  }
  memset(value, 0xff, sizeof(*value)); free(value);
}

static void sweep(void)
{
  YLispValue **v;
#ifdef GC_DEBUG
  for (v = &values; *v != NULL; v = &(*v)->next) {
    if (!(*v)->marked) {
      printf("Sweep %p: ", *v);
      ylisp_print(*v);
      printf("\n");
    }
  }
#endif
  for (v = &values; *v != NULL; v = &(*v)->next) {
    while (!(*v)->marked) {
      YLispValue *next = (*v)->next;
      free_value(*v);
      *v = next;
      if (next == NULL)
        return;
    }
    (*v)->marked = 0;
  }
}

static void run_gc(void)
{
  mark_roots();
  sweep();
  values_alloc_count = 0;
}

static void pin_variable(YLispValue **variable)
{
  GCPinnedVariable *pinned_var = ((GCPinnedVariable*)malloc(sizeof(GCPinnedVariable)));
  assert(pinned_var != NULL);
  pinned_var->variable = variable;
  pinned_var->next = pinned_vars; pinned_vars = pinned_var;
}

static void unpin_variable(YLispValue **variable)
{
  GCPinnedVariable **v;
  for (v = &pinned_vars; *v != NULL; v = &(*v)->next) {
    if ((*v)->variable == variable) {
      GCPinnedVariable *next = (*v)->next;
      free(*v);
      *v = next;
      return;
    }
  }
  assert(0);
}

static YLispValue *string_from_data(const char *data, size_t data_len)
{
  YLispValue *value;
  char *s = ((char*)(malloc(data_len + 1))); assert(s != NULL);
  memcpy(s, data, data_len); s[data_len] = '\0';

  value = ylisp_value(YLISP_STRING);
  value->v.s = s;
  return value;
}

YLispValue *ylisp_symbol_for_name(const char *name, size_t name_len)
{
  YLispValue *result, *symname;
  unsigned int i;

  for (i = 0; i < num_symbols; ++i) {
    symname = symbols[i]->v.symname;
    if (strlen(symname->v.s) == name_len
        && memcmp(symname->v.s, name, name_len) == 0) {
      return symbols[i];
    }
  }

  result = ylisp_value(YLISP_SYMBOL);
  result->v.symname = string_from_data(name, name_len);

  symbols = (YLispValue**)realloc(symbols, sizeof(*symbols) * (num_symbols + 1));
  assert(symbols != NULL);
  return symbols[num_symbols++] = result;
}

// ===============================================================================
// Lex
// ===============================================================================


/*
    theory all lisp is lower case, capitals interleaved will be intepretations
    E)rror, O)pen brace, C)lose brace, L)iteral,
*/
void ylisp_init_lexer(YLispLexer *lexer, char *buf)
{
  lexer->buffer = buf;
  lexer->position = 0;
}

static int is_sym_char(char c)
{
  return !isspace(c) && strchr("()[]{}\",'`;#|\\", c) == NULL;
}

#define c lexer->buffer[lexer->position]

static YLispToken ylisp_read_string(YLispLexer *lexer)
{
  unsigned int start = lexer->position;

  while (c != '\0' && c != '"') {
    ++lexer->position;
    LEX_DEBUG(c);
  }
  if (c == '\0') {
    LEX_DEBUG('E');
    return TOKEN_ERROR;
  }
  lexer->value = string_from_data(lexer->buffer + start,
                                  lexer->position - start);
  LEX_DEBUG(c);
  ++lexer->position;
  LEX_DEBUG('L');
  return TOKEN_LITERAL;
}

YLispToken ylisp_read_token(YLispLexer *lexer)
{
  while (c != '\0') {
    if (c == ';') {
      do {
        LEX_DEBUG(c);
        ++lexer->position;
        if (c == '\0') {
          LEX_DEBUG('E');
          return TOKEN_ERROR;
        }
      } while (c != '\n');
    } else if (!isspace(c)) {
      break;
    }
    LEX_DEBUG(c);
    ++lexer->position;
  }//

  if (c == '\0') {
    //LEX_DEBUG('F');
    return TOKEN_EOF;
  }

  switch (lexer->buffer[lexer->position++]) {
    case '.':
      LEX_DEBUG('P');
      return TOKEN_PERIOD;
    case '(':
      LEX_DEBUG('O');
      number_of_opened_parentheses++;
      return TOKEN_OPEN_PAREN;
    case ')':
      LEX_DEBUG('C');
      number_of_closed_parentheses++;
      return TOKEN_CLOSE_PAREN;
    case '\'':
      LEX_DEBUG('Q');
      return TOKEN_QUOTE;
    case '#':
      lexer->value = ylisp_number(YLISP_BOOLEAN, c == 't');
      if (c != 't' && c != 'f') {
        LEX_DEBUG('E');
        return TOKEN_ERROR;
      }
      LEX_DEBUG(c);
      ++lexer->position;
      LEX_DEBUG('L');
      return TOKEN_LITERAL;
    case '"':
      LEX_DEBUG('S');
      return ylisp_read_string(lexer);
    default:
      LEX_DEBUG(c);
      --lexer->position; break;
  }

  if (isdigit(c)) {
    lexer->value = ylisp_value(YLISP_NUMBER);
    lexer->value->v.i = 0;
    while (c != '\0' && isdigit(c)) {
      lexer->value->v.i = lexer->value->v.i * 10 + (c - '0');
      LEX_DEBUG(c);
      ++lexer->position;
    }
    LEX_DEBUG('L');
    return TOKEN_LITERAL;
  } else if (is_sym_char(c)) {
    unsigned int start = lexer->position;
    while (c != '\0' && is_sym_char(c)) {
      LEX_DEBUG(c);
      ++lexer->position;
    }
    lexer->value = ylisp_symbol_for_name(lexer->buffer + start,
                                         lexer->position - start);
    LEX_DEBUG('L');
    return TOKEN_LITERAL;
  } else {
    LEX_DEBUG('E');
    return TOKEN_ERROR;
  }
}

#undef c

// ===============================================================================
// Parse
// ===============================================================================



static YLispValue *parse_from_token(YLispLexer *lexer, YLispToken token);

static YLispValue *parse_list(YLispLexer *lexer)
{
  YLispValue *result, **rover = &result;

  for (;;) {
    YLispToken token = ylisp_read_token(lexer);
    if (token == TOKEN_EOF) {
      PARSE_DEBUG(".EOF"); // end of line actually
      break;
    }

    if (token == TOKEN_ERROR) {
      PARSE_DEBUG(".ERR");
      return NULL;
    }
    if (token == TOKEN_CLOSE_PAREN) {
      PARSE_DEBUG(".CLP");
      break;
    }

    // (x y . z) syntax:
    if (token == TOKEN_PERIOD) {
      token = ylisp_read_token(lexer);
      *rover = parse_from_token(lexer, token);
      token = ylisp_read_token(lexer);
      if (token != TOKEN_CLOSE_PAREN)
        return NULL;
      PARSE_DEBUG(".PER");
      return result;
    }
    *rover = ylisp_value(YLISP_CELL);
    CAR(*rover) = parse_from_token(lexer, token);
    rover = &CDR(*rover);
  }

  *rover = NULL;  // end of list
  return result;
}

static YLispValue *parse_quoted(YLispLexer *lexer)
{
  YLispToken token = ylisp_read_token(lexer);
  PARSE_DEBUG(".QTD");
  return ylisp_cons(keywords[KWD_QUOTE],
                    ylisp_cons(parse_from_token(lexer, token), NULL));
}

static YLispValue *parse_from_token(YLispLexer *lexer, YLispToken token)
{
  switch (token) {
    case TOKEN_OPEN_PAREN:
      PARSE_DEBUG(".OPP");
      return parse_list(lexer);
    case TOKEN_QUOTE:
      PARSE_DEBUG(".TQT");
      return parse_quoted(lexer);
    case TOKEN_LITERAL:
      PARSE_DEBUG(".LIT");
      return lexer->value;
    default:
      //PARSE_DEBUG(".TOK");
      //PARSE_DEBUG((int)token);
      break;
  }
  return NULL;
  //ylisp_number(YLISP_NUMBER,-1); // should be NULL
}

YLispValue *ylisp_parse(YLispLexer *lexer)
{
  YLispToken token = ylisp_read_token(lexer);
  return parse_from_token(lexer, token);
}


YLispValue *ylisp_parse_more(YLispLexer *lexer, YLispValue *codeUntilNow)
{
  YLispToken token = ylisp_read_token(lexer);
  return ylisp_cons(codeUntilNow, parse_from_token(lexer, token));
}

// ===============================================================================
// Eval
// ===============================================================================



YLispValue *ylisp_eval(YLispValue *context, YLispValue *code);

static YLispValue *eval_variable(YLispValue *context, YLispValue *var)
{
  YLispValue *v;

  while (context != NULL) {
    for (v = context->v.context.vars; v != NULL; v = CDR(v)) {
      if (CAR(CAR(v)) == var) {
        EVAL_DEBUG(".VAR");
        return CDR(CAR(v));
      }
    }
    context = context->v.context.parent;
  }

  static char UNDEFINED_VARIABLE[19] = {'U', 'n', 'd', 'e', 'f', 'i', 'n', 'e', 'd',
                                        ' ', 'v', 'a', 'r', 'i', 'a', 'b', 'l', 'e', '\0'
                                       };
  BLEAppend(UNDEFINED_VARIABLE);
  BLEAppend(var->v.symname->v.s);
  EVAL_DEBUG(".UNV");
  return NULL;
  //string_from_data(var->v.symname->v.s, strlen(var->v.symname->v.s));

}

static YLispValue *set_variable(YLispValue *context, YLispValue *name,
                                YLispValue *value)
{
  YLispValue *v;
  for (v = context->v.context.vars; v != NULL; v = CDR(v)) {
    if (CAR(CAR(v)) == name) {
      CDR(CAR(v)) = value;
      return value;
    }
  }
  context->v.context.vars = ylisp_cons(ylisp_cons(name, value),
                                       context->v.context.vars);
  return value;
}

static YLispValue *defer_eval(YLispValue *context, YLispValue *code)
{
  if (code->type != YLISP_CELL) {
    EVAL_DEBUG(".DYE");
    return ylisp_eval(context, code);
  }

  deferred_call.v.func.context = context;
  deferred_call.v.func.code = code;
  EVAL_DEBUG(".DEF");
  return &deferred_call;
}

static YLispValue *run_function_body(YLispValue *context, YLispValue *code)
{
  for (; code != NULL; code = CDR(code)) {
    if (CDR(code) == NULL) {
      EVAL_DEBUG(".DEV");
      return defer_eval(context, CAR(code));
    } else {
      ylisp_eval(context, CAR(code));
    }
  }
  EVAL_DEBUG(".NUL");
  return NULL;
}

static YLispValue *eval_func_args(YLispValue *context, YLispValue *code)
{

  YLispValue *args = NULL, **a = &args, *c;
  pin_variable(&args);
  for (c = code; c != NULL; c = CDR(c)) {
    *a = ylisp_value(YLISP_CELL);
    CAR(*a) = ylisp_eval(context, CAR(c));
    a = &CDR(*a);
  }
  *a = NULL;
  unpin_variable(&args);
  EVAL_DEBUG(".FAR");
  return args;
}

static YLispValue *eval_func_call(YLispValue *context, YLispValue *code)
{

  YLispValue *func = ylisp_eval(context, CAR(code));

  if (func->type == YLISP_BUILTIN) {
    YLispValue *result, *args = eval_func_args(context, CDR(code));
    pin_variable(&args);
    result = func->v.builtin(args);
    unpin_variable(&args);
    EVAL_DEBUG(".BUI");
    return result;
  } else if (func->type == YLISP_FUNCTION) {
    YLispValue *n, *c, *result;
    YLispValue *newcontext = ylisp_value(YLISP_CONTEXT);
    pin_variable(&newcontext);
    newcontext->v.context.parent = func->v.func.context;
    newcontext->v.context.vars = NULL;
    c = CDR(code);
    for (n = CAR(func->v.func.code); n != NULL; n = CDR(n)) {
      // varargs:
      if (n->type == YLISP_SYMBOL) {
        set_variable(newcontext, n,
                     eval_func_args(context, c));
        break;
      }
      set_variable(newcontext, CAR(n),
                   ylisp_eval(context, CAR(c)));
      c = CDR(c);
    }
    result = run_function_body(newcontext, CDR(func->v.func.code));
    unpin_variable(&newcontext);
    EVAL_DEBUG(".FUN");
    return result;
  } else {

    static char INVALID_FUNCTION_CALL[22] = {'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ',
                                             'f', 'u', 'n', 'c', 't', 'i', 'o', 'n', ' ',
                                             'c', 'a', 'l', 'l', '\0'
                                            };
    BLEAppend(INVALID_FUNCTION_CALL);
    EVAL_DEBUG(".IVF");
    return NULL;
  }
}

static YLispValue *eval_list_inner(YLispValue *context, YLispValue *code)
{
  YLispValue *first = CAR(code);

  // Special cases:
  if (first == keywords[KWD_QUOTE]) {
    return CAR(CDR(code));
  } else if (first == keywords[KWD_IF]) {
    YLispValue *val = ylisp_eval(context, CAR(CDR(code)));
    assert(val != NULL && (val->type == YLISP_NUMBER
                           || val->type == YLISP_BOOLEAN));
    if (val->v.i) {
      return defer_eval(context, CAR(CDR(CDR(code))));
    } else if (CDR(CDR(CDR(code))) != NULL) {
      return defer_eval(context, CAR(CDR(CDR(CDR(code)))));
    }
  } else if (first == keywords[KWD_COND]) {
    YLispValue *tests, *val;
    for (tests = CDR(code); tests != NULL; tests = CDR(tests)) {
      val = ylisp_eval(context, CAR(CAR(tests)));
      assert(val != NULL && (val->type == YLISP_NUMBER
                             || val->type == YLISP_BOOLEAN));
      if (val->v.i)
        return run_function_body(context,
                                 CDR(CAR(tests)));
    }
    return NULL;
  } else if (first == keywords[KWD_LAMBDA]) {
    YLispValue *result = ylisp_value(YLISP_FUNCTION);
    result->v.func.context = context;
    result->v.func.code = CDR(code);
    return result;
  } else if (first == keywords[KWD_DEFINE]) {
    // (define (func x) ...
    if (CAR(CDR(code))->type == YLISP_CELL) {
      YLispValue *func = ylisp_value(YLISP_FUNCTION);
      func->v.func.context = context;
      func->v.func.code = ylisp_cons(CDR(CAR(CDR(code))),
                                     CDR(CDR(code)));
      return set_variable(root_context, CAR(CAR(CDR(code))),
                          func);
    } else {
      return set_variable(root_context, CAR(CDR(code)),
                          ylisp_eval(context, CAR(CDR(CDR(code)))));
    }
  } else if (first == keywords[KWD_LET]) {
    YLispValue *newcontext = ylisp_value(YLISP_CONTEXT);
    YLispValue *vars, *result;
    newcontext->v.context.parent = context;
    pin_variable(&newcontext);
    for (vars = CAR(CDR(code)); vars != NULL; vars = CDR(vars)) {
      set_variable(newcontext, CAR(CAR(vars)),
                   ylisp_eval(newcontext,
                              CAR(CDR(CAR(vars)))));
    }
    result = run_function_body(newcontext, CDR(CDR(code)));
    unpin_variable(&newcontext);
    return result;
  } else if (first == keywords[KWD_BEGIN]) {
    return run_function_body(context, CDR(code));
  }

  return eval_func_call(context, code);
}

static YLispValue *eval_list(YLispValue *context, YLispValue *code)
{
  YLispValue *result = eval_list_inner(context, code);

  // Tail call optimization: expand deferred call.
  while (result == &deferred_call) {
    context = deferred_call.v.func.context;
    code = deferred_call.v.func.code;
    pin_variable(&context);
    result = eval_list_inner(context, code);
    unpin_variable(&context);
  }
  EVAL_DEBUG(".EVL");
  return result;
}

YLispValue *ylisp_eval(YLispValue *context, YLispValue *code)
{
  if (values_alloc_count > GC_ALLOC_TRIGGER)
    run_gc();
  EVAL_DEBUG(".YlE[");
  if (code == NULL) {
    EVAL_DEBUG("code is null");
  } else {
    EVAL_DEBUG((int)code->type);
    if ( ((int)code->type) > 10 )
    {
      EVAL_DEBUG('*');
      code_flagged_as_failing_to_be_processed_for_eval = true;
    }
    EVAL_DEBUG(']');
    switch (code->type) {
      case YLISP_SYMBOL:  // Variable
        EVAL_DEBUG(".SYM");
        return eval_variable(context, code);
      case YLISP_CELL:    // Function call or other.
        EVAL_DEBUG(".CEL");
        return eval_list(context, code);
      default:            // Literals.
        //Serial.print('o');
        break;
    }
  }
  return code;
}

// ===============================================================================
// Built in functions
// ===============================================================================

static YLispValue *builtin_add(YLispValue *args)
{
  return ylisp_number(YLISP_NUMBER,
                      CAR(args)->v.i + CAR(CDR(args))->v.i);
}

static YLispValue *builtin_sub(YLispValue *args)
{
  return ylisp_number(YLISP_NUMBER,
                      CAR(args)->v.i - CAR(CDR(args))->v.i);
}

static YLispValue *builtin_mul(YLispValue *args)
{
  return ylisp_number(YLISP_NUMBER,
                      CAR(args)->v.i * CAR(CDR(args))->v.i);
}

static YLispValue *builtin_div(YLispValue *args)
{
  return ylisp_number(YLISP_NUMBER,
                      CAR(args)->v.i / CAR(CDR(args))->v.i);
}

static YLispValue *builtin_eq(YLispValue *args)
{
  return ylisp_number(YLISP_BOOLEAN,
                      ylisp_equal(CAR(args), CAR(CDR(args))));
}

static YLispValue *builtin_lt(YLispValue *args)
{
  return ylisp_number(YLISP_BOOLEAN,
                      ylisp_lt(CAR(args), CAR(CDR(args))));
}

static YLispValue *builtin_and(YLispValue *args)
{
  return ylisp_number(YLISP_BOOLEAN,
                      CAR(args)->v.i && CAR(CDR(args))->v.i);
}

static YLispValue *builtin_or(YLispValue *args)
{
  return ylisp_number(YLISP_BOOLEAN,
                      CAR(args)->v.i || CAR(CDR(args))->v.i);
}

static YLispValue *builtin_not(YLispValue *args)
{
  return  ylisp_number(YLISP_BOOLEAN, !CAR(args)->v.i);
}

static YLispValue *builtin_car(YLispValue *args) {
  return CAR(CAR(args));
}
static YLispValue *builtin_cdr(YLispValue *args) {
  return CDR(CAR(args));
}

static YLispValue *builtin_cons(YLispValue *args)
{
  return ylisp_cons(CAR(args), CAR(CDR(args)));
}

static YLispValue *builtin_read(YLispValue *args)
{
  // TODO: Make this not suck
  YLispLexer lexer;
  char buf[256];
  printf("> "); fflush(stdout);
  fgets(buf, sizeof(buf), stdin); buf[sizeof(buf) - 1] = '\0';
  ylisp_init_lexer(&lexer, buf);
  return ylisp_parse(&lexer);
}

static YLispValue *builtin_eval(YLispValue *args)
{
  return ylisp_eval(root_context, CAR(args));
}

static YLispValue *builtin_print(YLispValue *args)
{
  ylisp_print(CAR(args)); printf("\n");
  return NULL;
}

static void define_builtin(char *name, YLispBuiltin callback)
{
  YLispValue *builtin = ylisp_value(YLISP_BUILTIN);
  builtin->v.builtin = callback;
  set_variable(root_context, ylisp_symbol_for_name(name, strlen(name)),
               builtin);
}

// ===============================================================================
// tim's functions
// ===============================================================================

static YLispValue *builtin_AD5252_Write(YLispValue *args)
{
  //----------------------------------------------------------------------------------------
  //void AD5252_Write(byte ch, byte x)    //To use send channel (1 or 2) and byte 0-255 = 0-1K ohms
  // USE: (AD5252 1 255) ; set optical resistance to max
  YLispValue* channel = ((args)->v.cell.car);
  YLispValue* rest = ((args)->v.cell.cdr);
  YLispValue* val =    ((rest)->v.cell.car);

  byte byte_channel = (byte)val->v.i;
  byte byte_val = (byte)val->v.i;

  //if (byte_channel <= 1)  {               //Error checking on channel value
  //  byte_channel = 1;                  //Channel = 1 for values <= 1
  //}
  //else byte_channel = 3;                //Channel = 3 for values >1 (as the AD5252 sees it)
  //byte 
  byte_channel=1;
  //byte byte_val = 0;
  
  //Serial.print('[');
  Wire.beginTransmission(Addr);   // Start I2C transmission
  //Serial.print(Addr); Serial.print(' ');
  Wire.write(byte_channel);// Send instruction for POT channel# 1 or 3
  //Serial.print(byte_channel); Serial.print(' ');
  Wire.write(byte_val);                  // Input resistance value passed in byte x
  //Serial.print(byte_val); Serial.print(' ');
  Wire.endTransmission();         // Stop I2C transmission
  //Serial.print(']');

  return NULL;
}

static YLispValue *builtin_FETControl(YLispValue *args)
{
  // USE: (fet 1 1) ; set FET channel 1 on
  YLispValue* channel = ((args)->v.cell.car);
  YLispValue* rest = ((args)->v.cell.cdr);
  YLispValue* val =    ((rest)->v.cell.car);

  int int_val = val->v.i;
  if (channel->v.i == 1) {

    if (int_val > 0) {
      digitalWrite(Ch1a, HIGH);
      digitalWrite(Ch1b, HIGH);
      digitalWrite(Green_LED, HIGH);
      BLEAppend("HI ");
      //BLEAppend(millis());
    } else {
      digitalWrite(Ch1a, LOW);
      digitalWrite(Ch1b, LOW);
      digitalWrite(Green_LED, LOW);
      BLEAppend("LO ");
      //BLEAppend(millis());
    }
  } else {
    Serial.print("undefined behaviour for channel:");
  }

  return NULL;
}
static YLispValue *builtin_digitalwrite(YLispValue *args)
{
  // USE: (dwrite pin value)
  // from Arduino.h?
  // digitalwrite

  YLispValue* pin = ((args)->v.cell.car);
  YLispValue* rest = ((args)->v.cell.cdr);
  YLispValue* value =    ((rest)->v.cell.car);

  int int_pin = (int)pin->v.i;
  if (int_pin >= 2 && int_pin <= 13) {
    Serial.print("pin:");
    Serial.print(int_pin);
    Serial.print(" ");
    if (value->v.i > 0) {
      Serial.print("High");
      digitalWrite(int_pin, HIGH);
    } else {
      Serial.print("Low");
      digitalWrite(int_pin, LOW);
    }

  } else {
    Serial.print("Pins out of bounds");
  }
  return NULL;
}

/*
static YLispValue *builtin_datetime(YLispValue *args)
{
  return
    ylisp_cons (
      ylisp_number(YLISP_NUMBER, year()),
      ylisp_cons (

        ylisp_number(YLISP_NUMBER, month()),
        ylisp_cons (

          ylisp_number(YLISP_NUMBER, day()),
          ylisp_cons (

            ylisp_number(YLISP_NUMBER, hour()),
            ylisp_cons (

              ylisp_number(YLISP_NUMBER, minute()),
              ylisp_cons (

                ylisp_number(YLISP_NUMBER, second()), NULL)
            )))));
}

static YLispValue *builtin_setdate(YLispValue *args)
{
  // USE: (setdate yy mm dd)
  // from CurieTime.h
  // setTime(int hour, int minute, int second, int day, int month, int year);

  YLispValue* yr = ((args)->v.cell.car);
  YLispValue* rest = ((args)->v.cell.cdr);
  YLispValue* mh =    ((rest)->v.cell.car);
  YLispValue* otherrest =  ((rest)->v.cell.cdr);
  YLispValue* dy =    ((otherrest)->v.cell.car);

  setTime(hour(), minute(), second(),
          (int)dy->v.i, (int)mh->v.i, ((int)yr->v.i) + 2000);

  return NULL;
}

static YLispValue *builtin_settime(YLispValue *args)
{
  // USE: (settime hh mm ss)
  // from CurieTime.h
  // setTime(int hour, int minute, int second, int day, int month, int year);

  YLispValue* hh = ((args)->v.cell.car);
  YLispValue* rest = ((args)->v.cell.cdr);
  YLispValue* mn =    ((rest)->v.cell.car);
  YLispValue* otherrest =  ((rest)->v.cell.cdr);
  YLispValue* ss =    ((otherrest)->v.cell.car);

  setTime((int)hh->v.i, (int)mn->v.i, (int)ss->v.i,
          day(), month(), year());

  return NULL;
}
*/
static YLispValue *builtin_memory(YLispValue *args)
{
  // ignore args
  // return ( free memory, stack memory . heap memory ) as integers
  // MemoryFree.h

  return
    ylisp_cons (
      //
      //  FreeMemory() Returns the number of bytes free in the heap,
      //  i.e. the number of bytes free to be allocated using malloc().
      //
      ylisp_number(YLISP_NUMBER, freeMemory()),

      ylisp_cons (

        //    Returns the number of bytes free in the stack,
        //    i.e. the space between the current stack frame and the end of the stack area.
        //    This function will return a negative number in the event of a stack overflow,
        //    representing the size of the overflow; for example,
        //    a return value of -20 means that the current stack frame is 20 bytes
        //    past the end of the stack area.

        ylisp_number(YLISP_NUMBER, freeStack()),

        ylisp_cons (

          //    Returns the number of bytes free in both the stack and the heap.
          //    If a stack overflow has occurred,
          //    only the number of bytes free in the heap is returned.

          ylisp_number(YLISP_NUMBER, freeHeap()),
          NULL
        )
      )
    );
}

static YLispValue *builtin_i2cw(YLispValue *args)
{
  // USE: (i2c-write address byte)
  // from Wire.h
  // Wire.begin(), beginTransmission, write, endTransmission, read

  YLispValue* fst = ((args)->v.cell.car);
  YLispValue* rst = ((args)->v.cell.cdr);
  YLispValue* snd =    ((rst)->v.cell.car);
  //YLispValue* other =  ((rst)->v.cell.cdr);
  //YLispValue* trd =    ((other)->v.cell.car);

  Wire.beginTransmission((int)fst->v.i);
  Wire.write((byte)(snd->v.i));
  Wire.endTransmission();

  return NULL;
}
// ===============================================================================
// initialisation
// ===============================================================================

void ylisp_init(void)
{
  unsigned int i;
  for (i = 0; i < NUM_KEYWORDS; ++i) {
    keywords[i] = ylisp_symbol_for_name(
                    keyword_names[i], strlen(keyword_names[i]));
  }
  root_context = ylisp_value(YLISP_CONTEXT);

  // integer operations
  define_builtin("add", builtin_add);
  define_builtin("sub", builtin_sub);
  define_builtin("mul", builtin_mul);
  define_builtin("div", builtin_div);

  // comparison operators
  define_builtin("lt", builtin_lt);
  define_builtin("eq", builtin_eq);
  define_builtin("and", builtin_and);
  define_builtin("or", builtin_or);
  define_builtin("not", builtin_not);

  //
  define_builtin("car", builtin_car);
  define_builtin("cdr", builtin_cdr);
  define_builtin("cons", builtin_cons);

  //  define_builtin("read", builtin_read);
  define_builtin("eval", builtin_eval);
  define_builtin("print", builtin_print);

  // tim's functions
  define_builtin("memory", builtin_memory);
/*
  define_builtin("setdate", builtin_setdate);
  define_builtin("settime", builtin_settime);
  define_builtin("datetime", builtin_datetime);
*/
  define_builtin("dw", builtin_digitalwrite);
  define_builtin("i2c-write", builtin_i2cw);
  define_builtin("fet", builtin_FETControl);
  define_builtin("ad5252", builtin_AD5252_Write);
  //code=NULL;
}
/*
  static char *read_file(char *filename)
  {
  char *result;
  unsigned int file_size;
  FILE *fs = fopen(filename, "r");
  if (fs == NULL) {
    perror("fopen");
    exit(-1);
  }
  fseek(fs, 0, SEEK_END);
  file_size = ftell(fs);
  fseek(fs, 0, SEEK_SET);
  result = (char*)malloc(file_size + 1); assert(result != NULL);
  fread(result, 1, file_size, fs);
  result[file_size] = '\0';
  return result;
  }
  static void process_file(char *filename)
  {
  YLispValue *code = NULL;
  YLispLexer lexer;
  char *infile;
  infile = read_file(filename);
  ylisp_init_lexer(&lexer, infile);
  pin_variable(&code);
  while ((code = ylisp_parse(&lexer)) != NULL) {
    //ylisp_print(code); printf("\n");
    ylisp_eval(root_context, code);
  }
  unpin_variable(&code);
  }
*/

#ifdef __cplusplus
} // extern "C"
#endif

YLispValue *code = NULL;
YLispValue *preexisting_code = NULL;
YLispLexer lexer;

// required for step detection
static void updateStepCount() {
  // get the step count:
  int stepCount = CurieIMU.getStepCount();

  // if the step count has changed, print it:
  if (stepCount != lastStepCount) {
    BLEAppend("Step:");

    char numBuffer[10];
    Itoa(stepCount, numBuffer, 10);
    BLEAppend(numBuffer);

    BLEAppend(" m");

    Itoa(millis(), numBuffer, 10);
    BLEAppend(numBuffer);

    // save the current count for comparison next check:
    lastStepCount = stepCount;
  }
}

// step detection
static void eventCallback(void) {
  if (CurieIMU.stepsDetected())
    updateStepCount();
}

void BLEAppend(const char *text) {

  // TODO handle when text is too long
  // TODO keep putting text in buffer until closing brace or 20 chars

  for (unsigned int i = 0; i < strlen((const char*)text); ++i) { // for every character in the message

    Serial.print((char)(text[i]));
    outputBuffer[filledTo++] = text[i]; // copy text to outputbuffer


// open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.print((char)text[ i ]);
        dataFile.close();
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }

  } // end stepping character by character through appended text

} // end BLEAppend

void BLEFlush() {

  txCharacteristic.setValue((const unsigned char*)outputBuffer, strlen(outputBuffer)); //  temp
  filledTo = 0;


  for (int i = 0 ; i < 20 ; i++) {
    outputBuffer[i] = '\0';
  }

}
void setup() {
  // put your setup code here, to run once:

  // ======================== leg stim =====================

  // pin initialisation

  //Set up all I/O as input or output
  pinMode(Blue_LED, OUTPUT);
  pinMode(Red_LED, OUTPUT);
  pinMode(Green_LED, OUTPUT);
  pinMode(Ch1a, OUTPUT);
  pinMode(Ch1b, OUTPUT);
  pinMode(Ch2a, OUTPUT);
  pinMode(Ch2b, OUTPUT);
  pinMode(BNO055_Int, INPUT);
  pinMode(Remote_LED, OUTPUT);
  pinMode(Remote_Pot, INPUT);
  pinMode(Remote_Sw1, INPUT);
  pinMode(Remote_Sw2, INPUT);

  //Set all outputs low to start off
  digitalWrite(Blue_LED, LOW);
  digitalWrite(Red_LED, LOW);
  digitalWrite(Green_LED, LOW);
  digitalWrite(Ch1a, LOW);  //Disables output from EMS
  digitalWrite(Ch1b, LOW);  //Disables output from EMS
  digitalWrite(Ch2a, LOW);  //Disables output from EMS
  digitalWrite(Ch2b, LOW);  //Disables output from EMS
  digitalWrite(Remote_LED, LOW);

  // -------------- serial -------------------

  // Open serial communications and wait for port to open:
  Serial.begin(115200); while (!Serial); // wait
  Serial.println("yoctoLisp 2013");

  // --------------- ble --------------------
  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  BLE.begin();

  /* Set a local name for the BLE device
      This name will appear in advertising packets
      and can be used by remote devices to identify this BLE device
      The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName(localName);

  BLE.setAdvertisedService(uartService);

  uartService.addCharacteristic(txCharacteristic);           // add the transmit characteristic
  uartService.addCharacteristic(rxCharacteristic);           // the the receive characteristic

  /*TODO could set tx and rx Characteritic value */
  BLE.addService(uartService);              // Add the uart service

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  BLE.advertise();

  // -------------------------- ylisp ------------------------------------

  ylisp_init();
  pin_variable(&code);

  // TRB give lexer non null values
  // original passes &lexer and ylisp_init_lexer accepts it as YLispLexer*
  static char buf[2] = {' ', '\0'};
  lexer.buffer = buf;
  lexer.position = 0;

  ylisp_init_lexer(&lexer, buf);

  unpin_variable(&code);

  // ---------------------------- imu ---------------------------

  // intialize the sensor:
  CurieIMU.begin();
  // turn on step detection mode:
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);
  // enable step counting:
  CurieIMU.setStepCountEnabled(true);

  // attach the eventCallback function as the
  // step event handler:
  CurieIMU.attachInterrupt(eventCallback);
  CurieIMU.interrupts(CURIE_IMU_STEP);  // turn on step detection

  Serial.println("IMU initialisation complete, waiting for events...");

  // --------------------------- i2c -------------------------

  Wire.begin();

  // --------------------------- sd card ---------------------

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  }
  Serial.println("card initialized.");

  // ------------------------- ad5252 -------------------------

  //----------------------------------------------------------------------------------------
  //void AD5252_Write(byte ch, byte x)    //To use send channel (1 or 2) and byte 0-255 = 0-1K ohms
  // USE: (AD5252 1 255) ; set optical resistance to max
  
  byte byte_channel = (byte)1;
  byte byte_val = (byte)10;

  Serial.print('[');
  Wire.beginTransmission(Addr);   // Start I2C transmission
  Serial.print(Addr); Serial.print(' ');
  Wire.write(byte_channel);// Send instruction for POT channel# 1 or 3
  Serial.print(byte_channel); Serial.print(' ');
  Wire.write(byte_val);                  // Input resistance value passed in byte x
  Serial.print(byte_val); Serial.print(' ');
  Wire.endTransmission();         // Stop I2C transmission
  Serial.print(']');
}

// ====================== loop helper functons =====================

int count_char(char *text, char search) {
  int result = 0;
  for (int i = 0; i < strlen(text); i++) {
    if (text[i] == search) {
      result++;
      //LOOP_DEBUG("text[i]==");
      //LOOP_DEBUG(text[i]);
      //LOOP_DEBUG("search==");
      //LOOP_DEBUG(search);
    }
  }
  return result;
}

void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  updateStepCount();
  // if a central is connected to peripheral:
  if (central) {

    while (central.connected()) {

      int preexistingCodeLength = strlen(LPE_code);
      int bufferLength = strlen(RXBuffer);
      int opensInPreexistingCode = count_char(LPE_code, '(');
      int openInBufferCode = count_char(RXBuffer, '(');
      int closeInPreexistingCode = count_char(LPE_code, ')');
      int closeInBufferCode = count_char(RXBuffer, ')');
      int opens = opensInPreexistingCode + openInBufferCode;
      int closes = closeInPreexistingCode + closeInBufferCode;

      // if there is new code (tracked in lex code)
      if (closes < opens) {
        if (preexistingCodeLength > 0) {
          LOOP_DEBUG("Opens:");
          LOOP_DEBUG(opens);
          LOOP_DEBUG("Closes:");
          LOOP_DEBUG(closes);

          // concatenate preexisting code, a space, and the new code.
          LOOP_DEBUG("CON!");
          // move existing code up by strlen(RXBuffer) spaces
          for (int i = 0; i < preexistingCodeLength; i++) {
            LPE_code[i + bufferLength] = LPE_code[i];
            LOOP_DEBUG(LPE_code[i]);
          }

          // add a ' '
          LPE_code[bufferLength] = ' ';
          LOOP_DEBUG(' ');
          // copy RXBuffer
          for (int i = 0; i < bufferLength; i++) {
            LPE_code[i] = RXBuffer[i];
            LOOP_DEBUG(RXBuffer[i]);
          }

          // add '\0';
          LPE_code[bufferLength + preexistingCodeLength] = '\0';

        } else {
          // first unbalanced code - copy RXBuffer into LPE_code
          for (int i = 0; i < bufferLength; i++) {
            LPE_code[i] = RXBuffer[i];
          }
        } // end pre existing code if then else statement
      } else { // balanced code
        if (opens > 0) {
          if (preexistingCodeLength == 0) {
            // copyRXBuffer to LPE_code
            for (int i = 0; i < bufferLength; i++) {
              LPE_code[i] = RXBuffer[i];
            }
            LPE_code[bufferLength] = '\0';
          }
          LOOP_DEBUG("standard run");
          // standard run interpreter code
          ylisp_init_lexer(&lexer, LPE_code);
          code = ylisp_parse(&lexer);

          ylisp_print(
            ylisp_eval(root_context, code)
          );

          if (code_flagged_as_failing_to_be_processed_for_eval) { // re-attempt once
            LOOP_DEBUG("re run!");
            // lex, parse failed

            ylisp_init_lexer(&lexer, RXBuffer);
            code = ylisp_parse(&lexer);

            ylisp_print(   ylisp_eval(root_context, code)  );
            // reset eval fail flag
            code_flagged_as_failing_to_be_processed_for_eval = false;
          }

          // blank LPE_code
          for (int i = 0; i < preexistingCodeLength; i++) {
            LPE_code[i] = '\0';
          }

          // blank RXBuffer
          for (int i = 0; i < bufferLength; i++) {
            RXBuffer[i] = '\0';
          }

        } // end balanced parentheses

        if (filledTo > 0) {
          BLEFlush();
          BLE.poll();
        } // end filledTo >0
      }
    } // end while connected
  } // end if central
} // end loop

void blePeripheralConnectHandler(BLEDevice central) { // central connected event handler
  Serial.print("Central Connected: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) { // central disconnected event handler
  Serial.print("Central Disconnected: ");
  Serial.println(central.address());
}

void rxCharacteristicWritten(BLECentral & central, BLECharacteristic & characteristic) {

  if (characteristic.value()) { // central wrote new value to characteristic

    int characteristiclength = characteristic.valueLength();
    for ( int idx = 0 ; idx < characteristiclength ; ++idx ) {

      // output the received string to Serial
      Serial.print( (char)characteristic[ idx ] );

      // copy it to the received text buffer
      RXBuffer[idx] = (char)characteristic[ idx ];


      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.print((char)characteristic[ idx ]);
        dataFile.close();
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }

    }

    RXBuffer[characteristiclength] = '\0'; // null terminate
    Serial.print('\n');
    ylisp_init_lexer(&lexer, RXBuffer);

  }
}

// simplest multi rx test
// rx:"(add 3 "
// rx:"5)"
// tx:"8"
