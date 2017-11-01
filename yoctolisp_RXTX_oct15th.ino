
/* 

  with thanks to Simon Howard - see https://github.com/fragglet/yoctolisp

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

       bug: if an attempt is made to transmit every time loop occurs happens,
       the Arduino hangs after less than a second.

        bug:  (define a 7)
              (define b 6)
              (add a b) --> Invalid function cal

        bug: (memory) the first time doesn't process  #949

        bug: when serial gets (8316 1880 6392) app gets ')'

        bug: setdatetime hh mm ss dd mm yy doesn't fit in 20 chars

        
               
*/

// porting it to Arduino 101 is taking considerably longer than a weekend

/*
  // TODO test
  // add sub mul div lt eq and or not car cdr cons read eval print memory
  // if quote cond lambda define let begin

  (add 5 4)
  (mul 3 8)
  (sub 5 9)
  (div 9 5)

*/

#include <CurieBLE.h> // needed for BLE uart 
#include <MemoryFree.h> // needed for (memory) --> (free stack . heap)
#include <CurieTime.h> // needed for (datetime hh mm ss dd mm yy)

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

/* BLE */

const char* localName = "Lisp";
static const int BLE_MAX_LENGTH = 20;
char RXBuffer[BLE_MAX_LENGTH + 1];

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
int filledTo = -1;

#define mirrorREPLToSerial 1 // if non zero everything sent to BLE tx gets sent to serial too

/* end TRB yocto addition */

/* end BLE definitions */

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
  "if", "cond", "quote", "lambda", "let", "define", "begin"
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
  }
  if (c == '\0')
    return TOKEN_ERROR;

  lexer->value = string_from_data(lexer->buffer + start,
                                  lexer->position - start);
  ++lexer->position;
  return TOKEN_LITERAL;
}

YLispToken ylisp_read_token(YLispLexer *lexer)
{
  while (c != '\0') {
    if (c == ';') {
      do {
        ++lexer->position;
        if (c == '\0')
          return TOKEN_ERROR;
      } while (c != '\n');
    } else if (!isspace(c)) {
      break;
    }
    ++lexer->position;
  }

  if (c == '\0')
    return TOKEN_EOF;

  switch (lexer->buffer[lexer->position++]) {
    case '.': return TOKEN_PERIOD;
    case '(': return TOKEN_OPEN_PAREN;
    case ')': return TOKEN_CLOSE_PAREN;
    case '\'': return TOKEN_QUOTE;
    case '#':
      lexer->value = ylisp_number(YLISP_BOOLEAN, c == 't');
      if (c != 't' && c != 'f')
        return TOKEN_ERROR;
      ++lexer->position;
      return TOKEN_LITERAL;
    case '"': return ylisp_read_string(lexer);
    default: --lexer->position; break;
  }

  if (isdigit(c)) {
    lexer->value = ylisp_value(YLISP_NUMBER);
    lexer->value->v.i = 0;
    while (c != '\0' && isdigit(c)) {
      lexer->value->v.i = lexer->value->v.i * 10 + (c - '0');
      ++lexer->position;
    }

    return TOKEN_LITERAL;
  } else if (is_sym_char(c)) {
    unsigned int start = lexer->position;
    while (c != '\0' && is_sym_char(c))
      ++lexer->position;
    lexer->value = ylisp_symbol_for_name(lexer->buffer + start,
                                         lexer->position - start);
    return TOKEN_LITERAL;
  } else {
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
    if (token == TOKEN_EOF || token == TOKEN_ERROR)
      return NULL;
    if (token == TOKEN_CLOSE_PAREN)
      break;
    // (x y . z) syntax:
    if (token == TOKEN_PERIOD) {
      token = ylisp_read_token(lexer);
      *rover = parse_from_token(lexer, token);
      token = ylisp_read_token(lexer);
      if (token != TOKEN_CLOSE_PAREN)
        return NULL;
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

  return ylisp_cons(keywords[KWD_QUOTE],
                    ylisp_cons(parse_from_token(lexer, token), NULL));
}

static YLispValue *parse_from_token(YLispLexer *lexer, YLispToken token)
{
  switch (token) {
    case TOKEN_OPEN_PAREN:
      return parse_list(lexer);
    case TOKEN_QUOTE:
      return parse_quoted(lexer);
    case TOKEN_LITERAL:
      return lexer->value;
    default:
      break;
  }
  return NULL;
}

YLispValue *ylisp_parse(YLispLexer *lexer)
{
  YLispToken token = ylisp_read_token(lexer);
  return parse_from_token(lexer, token);
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
  return string_from_data(var->v.symname->v.s, strlen(var->v.symname->v.s));

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
    return ylisp_eval(context, code);
  }

  deferred_call.v.func.context = context;
  deferred_call.v.func.code = code;
  return &deferred_call;
}

static YLispValue *run_function_body(YLispValue *context, YLispValue *code)
{
  for (; code != NULL; code = CDR(code)) {
    if (CDR(code) == NULL) {
      return defer_eval(context, CAR(code));
    } else {
      ylisp_eval(context, CAR(code));
    }
  }
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
    return result;
  } else {

    static char INVALID_FUNCTION_CALL[22] = {'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ',
                                             'f', 'u', 'n', 'c', 't', 'i', 'o', 'n', ' ',
                                             'c', 'a', 'l', 'l', '\0'
                                            };
    BLEAppend(INVALID_FUNCTION_CALL);
    return CAR(func->v.func.code);
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

  return result;
}

YLispValue *ylisp_eval(YLispValue *context, YLispValue *code)
{
  if (values_alloc_count > GC_ALLOC_TRIGGER)
    run_gc();
  switch (code->type) {
    case YLISP_SYMBOL:  // Variable
      return eval_variable(context, code);
    case YLISP_CELL:    // Function call or other.
      return eval_list(context, code);
    default:            // Literals.
      break;
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


static YLispValue *builtin_setdatetime(YLispValue *args)
{
  // USE: (setdatetimehh mm ss dd mm yy)
  // from CurieTime.h
  // setTime(int hour, int minute, int second, int day, int month, int year);
  // TODO find a way for this to evaluate once 
  
  YLispValue* hr =    builtin_cdr(args);
  YLispValue* mn =    builtin_cdr(hr);
  YLispValue* sc =    builtin_cdr(mn);
  YLispValue* dy =    builtin_cdr(sc);
  YLispValue* mt =    builtin_cdr(dy);
  YLispValue* yr =    builtin_cdr(mt); 
 
  setTime((int)hr->v.i, (int)mn->v.i, (int)sc->v.i, 
          (int)dy->v.i, (int)mt->v.i, (int)yr->v.i);
  
  return NULL;
}

static YLispValue *builtin_memory(YLispValue *args)
{
  // ignore args
  // return ( free memory, stack memory . heap memory ) as integers
  // MemoryFree.h
  
  return
    ylisp_cons (
      /*
        FreeMemory() Returns the number of bytes free in the heap,
        i.e. the number of bytes free to be allocated using malloc().
      */
      ylisp_number(YLISP_NUMBER, freeMemory()),

      ylisp_cons (
        /*
          Returns the number of bytes free in the stack,
          i.e. the space between the current stack frame and the end of the stack area.
          This function will return a negative number in the event of a stack overflow,
          representing the size of the overflow; for example,
          a return value of -20 means that the current stack frame is 20 bytes
          past the end of the stack area.
        */
        ylisp_number(YLISP_NUMBER, freeStack()),

       ylisp_cons (
        /*
          Returns the number of bytes free in both the stack and the heap.
          If a stack overflow has occurred,
          only the number of bytes free in the heap is returned.
        */
        ylisp_number(YLISP_NUMBER, freeHeap()),
        NULL
        )
      )
    );
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
  define_builtin("add", builtin_add);
  define_builtin("sub", builtin_sub);
  define_builtin("mul", builtin_mul);
  define_builtin("div", builtin_div);
  define_builtin("lt", builtin_lt);
  define_builtin("eq", builtin_eq);
  define_builtin("and", builtin_and);
  define_builtin("or", builtin_or);
  define_builtin("not", builtin_not);
  define_builtin("car", builtin_car);
  define_builtin("cdr", builtin_cdr);
  define_builtin("cons", builtin_cons);
//  define_builtin("read", builtin_read);
  define_builtin("eval", builtin_eval);
  define_builtin("print", builtin_print);
  define_builtin("memory", builtin_memory);
  define_builtin("setdatatime", builtin_setdatetime);
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
YLispLexer lexer;

void BLEAppend(const char *text) {

  // TODO handle when text is too long
  // TODO keep putting text in buffer until closing brace or 20 chars

  for (unsigned int i = 0; i < strlen((const char*)text); ++i) { // for every character in the message

    Serial.print((char)(text[i]));
    outputBuffer[++filledTo] = text[i]; // copy text to outputbuffer

    if (filledTo > BLE_MAX_LENGTH || text[i] == ')') { // ready to transmit?
      txCharacteristic.setValue((const unsigned char*)text, strlen(text)); //  temp
      filledTo = -1;
    } // end buffer ready to send
  } // end stepping character by character through appended text
  //txCharacteristic.setValue((const unsigned char*)text, strlen(text)); //  temp

  BLE.poll();// also temp

} // end BLEAppend

void setup() {
  // put your setup code here, to run once:

  // Open serial communications and wait for port to open:
  Serial.begin(115200); while (!Serial); // wait
  Serial.println("yoctoLisp 2013");

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

  ylisp_init();
  pin_variable(&code);

  // TRB give lexer non null values
  // original passes &lexer and ylisp_init_lexer accepts it as YLispLexer*
  static char buf[2] = {' ', '\0'};
  lexer.buffer = buf;
  lexer.position = 0;

  ylisp_init_lexer(&lexer, buf);

  unpin_variable(&code);

}

void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {

    while (central.connected()) {

      code = ylisp_parse(&lexer);
      
      /*if (code != NULL) {
        Serial.print("DEBUG:{");
        ylisp_print(code);
        Serial.print("}");
      }*/

      if (code != NULL) {
        ylisp_print(
          ylisp_eval(root_context, code)
        );
      }

      BLE.poll();

    }
  }
}

void blePeripheralConnectHandler(BLEDevice central) { // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) { // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void rxCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {

  // central wrote new value to characteristic
  if (characteristic.value()) {

    int characteristiclength = characteristic.valueLength();
    for ( int idx = 0 ; idx < characteristiclength ; ++idx ) {

      // output the received string to Serial
      Serial.print( (char)characteristic[ idx ] );

      // copy it to the received text buffer
      RXBuffer[idx] = (char)characteristic[ idx ];
    }

    // experimenting with what it takes to correctly stop parsing and evaluating
    // between BLE RX characteristics.
    RXBuffer[characteristiclength] = '\0';
    Serial.print('\n');
    ylisp_init_lexer(&lexer, RXBuffer);
    /*
       TRB - this assumes only one message at a time and parsing is possible before nxt one.
       concern- am I limited t 20 characters in total for an open and close brace?
    */
  }
}


