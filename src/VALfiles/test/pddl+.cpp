/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.3"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     OPEN_BRAC = 258,
     CLOSE_BRAC = 259,
     OPEN_SQ = 260,
     CLOSE_SQ = 261,
     DEFINE = 262,
     PDDLDOMAIN = 263,
     REQS = 264,
     EQUALITY = 265,
     STRIPS = 266,
     ADL = 267,
     NEGATIVE_PRECONDITIONS = 268,
     TYPING = 269,
     DISJUNCTIVE_PRECONDS = 270,
     EXT_PRECS = 271,
     UNIV_PRECS = 272,
     QUANT_PRECS = 273,
     COND_EFFS = 274,
     FLUENTS = 275,
     TIME = 276,
     DURATIVE_ACTIONS = 277,
     DURATION_INEQUALITIES = 278,
     CONTINUOUS_EFFECTS = 279,
     DERIVED_PREDICATES = 280,
     TIMED_INITIAL_LITERALS = 281,
     PREFERENCES = 282,
     CONSTRAINTS = 283,
     ACTION = 284,
     PROCESS = 285,
     EVENT = 286,
     DURATIVE_ACTION = 287,
     DERIVED = 288,
     CONSTANTS = 289,
     PREDS = 290,
     FUNCTIONS = 291,
     TYPES = 292,
     ARGS = 293,
     PRE = 294,
     CONDITION = 295,
     PREFERENCE = 296,
     START_PRE = 297,
     END_PRE = 298,
     EFFECTS = 299,
     INITIAL_EFFECT = 300,
     FINAL_EFFECT = 301,
     INVARIANT = 302,
     DURATION = 303,
     AT_START = 304,
     AT_END = 305,
     OVER_ALL = 306,
     AND = 307,
     OR = 308,
     EXISTS = 309,
     FORALL = 310,
     IMPLY = 311,
     NOT = 312,
     WHEN = 313,
     WHENEVER = 314,
     EITHER = 315,
     PROBLEM = 316,
     FORDOMAIN = 317,
     INITIALLY = 318,
     OBJECTS = 319,
     GOALS = 320,
     EQ = 321,
     LENGTH = 322,
     SERIAL = 323,
     PARALLEL = 324,
     METRIC = 325,
     MINIMIZE = 326,
     MAXIMIZE = 327,
     HASHT = 328,
     DURATION_VAR = 329,
     TOTAL_TIME = 330,
     INCREASE = 331,
     DECREASE = 332,
     SCALE_UP = 333,
     SCALE_DOWN = 334,
     ASSIGN = 335,
     GREATER = 336,
     GREATEQ = 337,
     LESS = 338,
     LESSEQ = 339,
     Q = 340,
     COLON = 341,
     ALWAYS = 342,
     SOMETIME = 343,
     WITHIN = 344,
     ATMOSTONCE = 345,
     SOMETIMEAFTER = 346,
     SOMETIMEBEFORE = 347,
     ALWAYSWITHIN = 348,
     HOLDDURING = 349,
     HOLDAFTER = 350,
     ISVIOLATED = 351,
     BOGUS = 352,
     NAME = 353,
     FUNCTION_SYMBOL = 354,
     INTVAL = 355,
     FLOATVAL = 356,
     AT_TIME = 357,
     PLUS = 358,
     HYPHEN = 359,
     DIV = 360,
     MUL = 361,
     UMINUS = 362
   };
#endif
/* Tokens.  */
#define OPEN_BRAC 258
#define CLOSE_BRAC 259
#define OPEN_SQ 260
#define CLOSE_SQ 261
#define DEFINE 262
#define PDDLDOMAIN 263
#define REQS 264
#define EQUALITY 265
#define STRIPS 266
#define ADL 267
#define NEGATIVE_PRECONDITIONS 268
#define TYPING 269
#define DISJUNCTIVE_PRECONDS 270
#define EXT_PRECS 271
#define UNIV_PRECS 272
#define QUANT_PRECS 273
#define COND_EFFS 274
#define FLUENTS 275
#define TIME 276
#define DURATIVE_ACTIONS 277
#define DURATION_INEQUALITIES 278
#define CONTINUOUS_EFFECTS 279
#define DERIVED_PREDICATES 280
#define TIMED_INITIAL_LITERALS 281
#define PREFERENCES 282
#define CONSTRAINTS 283
#define ACTION 284
#define PROCESS 285
#define EVENT 286
#define DURATIVE_ACTION 287
#define DERIVED 288
#define CONSTANTS 289
#define PREDS 290
#define FUNCTIONS 291
#define TYPES 292
#define ARGS 293
#define PRE 294
#define CONDITION 295
#define PREFERENCE 296
#define START_PRE 297
#define END_PRE 298
#define EFFECTS 299
#define INITIAL_EFFECT 300
#define FINAL_EFFECT 301
#define INVARIANT 302
#define DURATION 303
#define AT_START 304
#define AT_END 305
#define OVER_ALL 306
#define AND 307
#define OR 308
#define EXISTS 309
#define FORALL 310
#define IMPLY 311
#define NOT 312
#define WHEN 313
#define WHENEVER 314
#define EITHER 315
#define PROBLEM 316
#define FORDOMAIN 317
#define INITIALLY 318
#define OBJECTS 319
#define GOALS 320
#define EQ 321
#define LENGTH 322
#define SERIAL 323
#define PARALLEL 324
#define METRIC 325
#define MINIMIZE 326
#define MAXIMIZE 327
#define HASHT 328
#define DURATION_VAR 329
#define TOTAL_TIME 330
#define INCREASE 331
#define DECREASE 332
#define SCALE_UP 333
#define SCALE_DOWN 334
#define ASSIGN 335
#define GREATER 336
#define GREATEQ 337
#define LESS 338
#define LESSEQ 339
#define Q 340
#define COLON 341
#define ALWAYS 342
#define SOMETIME 343
#define WITHIN 344
#define ATMOSTONCE 345
#define SOMETIMEAFTER 346
#define SOMETIMEBEFORE 347
#define ALWAYSWITHIN 348
#define HOLDDURING 349
#define HOLDAFTER 350
#define ISVIOLATED 351
#define BOGUS 352
#define NAME 353
#define FUNCTION_SYMBOL 354
#define INTVAL 355
#define FLOATVAL 356
#define AT_TIME 357
#define PLUS 358
#define HYPHEN 359
#define DIV 360
#define MUL 361
#define UMINUS 362




/* Copy the first part of user declarations.  */
#line 17 "../pddl+.yacc"

/*
Error reporting:
Intention is to provide error token on most bracket expressions,
so synchronisation can occur on next CLOSE_BRAC.
Hence error should be generated for innermost expression containing error.
Expressions which cause errors return a NULL values, and parser
always attempts to carry on.
This won't behave so well if CLOSE_BRAC is missing.

Naming conventions:
Generally, the names should be similar to the PDDL2.1 spec.
During development, they have also been based on older PDDL specs,
older PDDL+ and TIM parsers, and this shows in places.

All the names of fields in the semantic value type begin with t_
Corresponding categories in the grammar begin with c_
Corresponding classes have no prefix.

PDDL grammar       yacc grammar      type of corresponding semantic val.  

thing+             c_things          thing_list
(thing+)           c_thing_list      thing_list

*/

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <ctype.h>

// This is now copied locally to avoid relying on installation 
// of flex++.

//#include "FlexLexer.h"
//#include <FlexLexer.h>

#include "ptree.h"
#include "parse_error.h"

#define YYDEBUG 1 

int yyerror(char *);


extern int yylex();

using namespace VAL;



/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 68 "../pddl+.yacc"
{
    parse_category* t_parse_category;

    effect_lists* t_effect_lists;
    effect* t_effect;
    simple_effect* t_simple_effect;
    cond_effect*   t_cond_effect;
    forall_effect* t_forall_effect;
    timed_effect* t_timed_effect;

    quantifier t_quantifier;
    metric_spec*  t_metric;
    optimization t_optimization;

    symbol* t_symbol;
    var_symbol*   t_var_symbol;
    pddl_type*    t_type;
    pred_symbol*  t_pred_symbol;
    func_symbol*  t_func_symbol;
    const_symbol* t_const_symbol;

    parameter_symbol_list* t_parameter_symbol_list;
    var_symbol_list* t_var_symbol_list;
    const_symbol_list* t_const_symbol_list;
    pddl_type_list* t_type_list;

    proposition* t_proposition;
    pred_decl* t_pred_decl;
    pred_decl_list* t_pred_decl_list;
    func_decl* t_func_decl;
    func_decl_list* t_func_decl_list;

    goal* t_goal;
    con_goal * t_con_goal;
    goal_list* t_goal_list;

    func_term* t_func_term;
    assignment* t_assignment;
    expression* t_expression;
    num_expression* t_num_expression;
    assign_op t_assign_op;
    comparison_op t_comparison_op;

    structure_def* t_structure_def;
    structure_store* t_structure_store;

    action* t_action_def;
    event* t_event_def;
    process* t_process_def;
    durative_action* t_durative_action_def;
    derivation_rule* t_derivation_rule;

    problem* t_problem;
    length_spec* t_length_spec;

    domain* t_domain;    

    pddl_req_flag t_pddl_req_flag;

    plan* t_plan;
    plan_step* t_step;

    int ival;
    double fval;

    char* cp;
    int t_dummy;

    var_symbol_table * vtab;
}
/* Line 187 of yacc.c.  */
#line 432 "pddl+.cpp"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 216 of yacc.c.  */
#line 445 "pddl+.cpp"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int i)
#else
static int
YYID (i)
    int i;
#endif
{
  return i;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack, Stack, yysize);				\
	Stack = &yyptr->Stack;						\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  17
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   908

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  108
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  122
/* YYNRULES -- Number of rules.  */
#define YYNRULES  329
/* YYNRULES -- Number of states.  */
#define YYNSTATES  768

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   362

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,    82,    83,    84,
      85,    86,    87,    88,    89,    90,    91,    92,    93,    94,
      95,    96,    97,    98,    99,   100,   101,   102,   103,   104,
     105,   106,   107
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     5,     7,     9,    15,    20,    23,    26,
      29,    32,    35,    38,    40,    45,    50,    55,    58,    59,
      62,    64,    69,    73,    75,    77,    79,    81,    84,    85,
      90,    94,    96,   101,   106,   108,   112,   113,   118,   123,
     125,   128,   129,   132,   133,   138,   143,   145,   148,   152,
     153,   155,   157,   159,   161,   166,   168,   170,   173,   174,
     177,   178,   185,   188,   191,   194,   195,   200,   203,   206,
     209,   210,   212,   214,   216,   218,   220,   225,   227,   229,
     231,   233,   236,   239,   242,   243,   248,   253,   258,   266,
     272,   278,   280,   282,   285,   286,   291,   296,   302,   308,
     312,   318,   324,   328,   333,   341,   347,   349,   352,   353,
     358,   360,   362,   364,   366,   369,   372,   375,   376,   382,
     388,   394,   400,   406,   412,   418,   423,   426,   427,   429,
     432,   434,   436,   442,   448,   454,   460,   465,   472,   482,
     492,   494,   496,   498,   500,   503,   504,   509,   511,   516,
     518,   526,   532,   538,   544,   550,   556,   562,   567,   573,
     579,   585,   591,   593,   595,   601,   607,   609,   611,   613,
     618,   623,   625,   630,   635,   637,   639,   641,   643,   645,
     647,   649,   651,   656,   662,   667,   675,   677,   682,   688,
     693,   701,   704,   706,   711,   717,   720,   722,   727,   735,
     740,   745,   750,   756,   761,   767,   773,   780,   787,   793,
     795,   800,   805,   810,   816,   824,   832,   838,   841,   843,
     845,   847,   852,   857,   862,   867,   872,   877,   882,   887,
     892,   897,   902,   905,   907,   909,   911,   913,   915,   917,
     919,   925,   938,   943,   956,   961,   974,   979,   991,   996,
    1000,  1004,  1005,  1007,  1012,  1015,  1016,  1021,  1026,  1031,
    1037,  1042,  1044,  1046,  1048,  1050,  1052,  1054,  1056,  1058,
    1060,  1062,  1064,  1066,  1068,  1070,  1072,  1074,  1076,  1078,
    1080,  1082,  1084,  1089,  1094,  1107,  1113,  1116,  1119,  1122,
    1125,  1128,  1131,  1134,  1135,  1140,  1145,  1147,  1152,  1158,
    1163,  1171,  1177,  1183,  1185,  1187,  1191,  1193,  1195,  1197,
    1202,  1206,  1210,  1214,  1218,  1222,  1224,  1227,  1229,  1232,
    1235,  1239,  1243,  1244,  1248,  1250,  1255,  1257,  1262,  1264
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int16 yyrhs[] =
{
     109,     0,    -1,   110,    -1,   212,    -1,   225,    -1,     3,
       7,   112,   111,     4,    -1,     3,     7,   112,     1,    -1,
     113,   111,    -1,   211,   111,    -1,   210,   111,    -1,   192,
     111,    -1,   193,   111,    -1,   194,   111,    -1,   196,    -1,
       3,     8,    98,     4,    -1,     3,     9,   114,     4,    -1,
       3,     9,     1,     4,    -1,   114,   209,    -1,    -1,   116,
     115,    -1,   116,    -1,     3,   117,   123,     4,    -1,     3,
       1,     4,    -1,    98,    -1,    66,    -1,    98,    -1,    98,
      -1,   120,   121,    -1,    -1,     3,   122,   123,     4,    -1,
       3,     1,     4,    -1,    98,    -1,   124,   104,   136,   123,
      -1,   124,   104,   134,   123,    -1,   124,    -1,    85,   130,
     124,    -1,    -1,   127,   104,   136,   125,    -1,   127,   104,
     134,   125,    -1,   127,    -1,   132,   126,    -1,    -1,   133,
     127,    -1,    -1,   137,   104,   136,   128,    -1,   137,   104,
     134,   128,    -1,   137,    -1,   129,   132,    -1,   129,    85,
     131,    -1,    -1,    98,    -1,    98,    -1,    98,    -1,    98,
      -1,     3,    60,   138,     4,    -1,    98,    -1,    98,    -1,
     137,   135,    -1,    -1,   138,   136,    -1,    -1,   139,     3,
      66,   175,   174,     4,    -1,   139,   168,    -1,   139,   167,
      -1,   139,   140,    -1,    -1,     3,   102,   139,     4,    -1,
     143,   141,    -1,   170,   141,    -1,   169,   141,    -1,    -1,
     146,    -1,   166,    -1,   165,    -1,   170,    -1,   169,    -1,
       3,    52,   145,     4,    -1,   144,    -1,   165,    -1,   166,
      -1,   171,    -1,   145,   165,    -1,   145,   166,    -1,   145,
     171,    -1,    -1,     3,    52,   141,     4,    -1,     3,    52,
       1,     4,    -1,     3,    52,   148,     4,    -1,     3,   187,
       3,   123,     4,   147,     4,    -1,     3,    58,   205,   147,
       4,    -1,     3,    59,   185,   151,     4,    -1,   149,    -1,
     171,    -1,   148,   147,    -1,    -1,     3,    49,   153,     4,
      -1,     3,    50,   153,     4,    -1,     3,    76,   175,   173,
       4,    -1,     3,    77,   175,   173,     4,    -1,     3,     1,
       4,    -1,     3,    76,   175,   173,     4,    -1,     3,    77,
     175,   173,     4,    -1,     3,     1,     4,    -1,     3,    52,
     152,     4,    -1,     3,   187,     3,   123,     4,   151,     4,
      -1,     3,    59,   185,   151,     4,    -1,   150,    -1,   152,
     151,    -1,    -1,     3,    52,   155,     4,    -1,   154,    -1,
     165,    -1,   166,    -1,   156,    -1,   155,   165,    -1,   155,
     166,    -1,   155,   156,    -1,    -1,     3,    80,   175,   159,
       4,    -1,     3,    76,   175,   159,     4,    -1,     3,    77,
     175,   159,     4,    -1,     3,    78,   175,   159,     4,    -1,
       3,    79,   175,   159,     4,    -1,     3,    76,   175,   173,
       4,    -1,     3,    77,   175,   173,     4,    -1,     3,    52,
     158,     4,    -1,   158,   157,    -1,    -1,   160,    -1,    85,
      74,    -1,   174,    -1,   175,    -1,     3,   103,   159,   159,
       4,    -1,     3,   104,   159,   159,     4,    -1,     3,   106,
     159,   159,     4,    -1,     3,   105,   159,   159,     4,    -1,
       3,    52,   164,     4,    -1,     3,   162,    85,    74,   163,
       4,    -1,     3,    49,     3,   162,    85,    74,   163,     4,
       4,    -1,     3,    50,     3,   162,    85,    74,   163,     4,
       4,    -1,    84,    -1,    82,    -1,    66,    -1,   172,    -1,
     164,   161,    -1,    -1,     3,    57,   189,     4,    -1,   189,
      -1,     3,    57,   191,     4,    -1,   191,    -1,     3,   187,
       3,   123,     4,   142,     4,    -1,     3,    58,   185,   141,
       4,    -1,     3,    80,   175,   172,     4,    -1,     3,    76,
     175,   172,     4,    -1,     3,    77,   175,   172,     4,    -1,
       3,    78,   175,   172,     4,    -1,     3,    79,   175,   172,
       4,    -1,     3,   104,   172,     4,    -1,     3,   103,   172,
     172,     4,    -1,     3,   104,   172,   172,     4,    -1,     3,
     106,   172,   172,     4,    -1,     3,   105,   172,   172,     4,
      -1,   174,    -1,   175,    -1,     3,   106,    73,   172,     4,
      -1,     3,   106,   172,    73,     4,    -1,    73,    -1,   100,
      -1,   101,    -1,     3,    99,   129,     4,    -1,     3,    98,
     129,     4,    -1,    99,    -1,     3,    99,   129,     4,    -1,
       3,    98,   129,     4,    -1,    99,    -1,    81,    -1,    82,
      -1,    83,    -1,    84,    -1,    66,    -1,   182,    -1,   185,
      -1,     3,    41,   184,     4,    -1,     3,    41,    98,   184,
       4,    -1,     3,    52,   181,     4,    -1,     3,   187,     3,
     123,     4,   180,     4,    -1,   184,    -1,     3,    41,   184,
       4,    -1,     3,    41,    98,   184,     4,    -1,     3,    52,
     181,     4,    -1,     3,   187,     3,   123,     4,   180,     4,
      -1,   181,   179,    -1,   179,    -1,     3,    41,   185,     4,
      -1,     3,    41,    98,   185,     4,    -1,   183,   184,    -1,
     184,    -1,     3,    52,   183,     4,    -1,     3,   187,     3,
     123,     4,   184,     4,    -1,     3,    50,   185,     4,    -1,
       3,    87,   185,     4,    -1,     3,    88,   185,     4,    -1,
       3,    89,   174,   185,     4,    -1,     3,    90,   185,     4,
      -1,     3,    91,   185,   185,     4,    -1,     3,    92,   185,
     185,     4,    -1,     3,    93,   174,   185,   185,     4,    -1,
       3,    94,   174,   174,   185,     4,    -1,     3,    95,   174,
     185,     4,    -1,   189,    -1,     3,    57,   185,     4,    -1,
       3,    52,   186,     4,    -1,     3,    53,   186,     4,    -1,
       3,    56,   185,   185,     4,    -1,     3,   187,     3,   123,
       4,   185,     4,    -1,     3,   188,     3,   123,     4,   185,
       4,    -1,     3,   177,   172,   172,     4,    -1,   186,   185,
      -1,   185,    -1,    55,    -1,    54,    -1,     3,   118,   129,
       4,    -1,     3,   118,   123,     4,    -1,     3,   119,   129,
       4,    -1,     3,    35,   115,     4,    -1,     3,    35,     1,
       4,    -1,     3,    36,   120,     4,    -1,     3,    36,     1,
       4,    -1,     3,    28,   184,     4,    -1,     3,    28,     1,
       4,    -1,     3,    28,   179,     4,    -1,     3,    28,     1,
       4,    -1,   196,   197,    -1,   197,    -1,   200,    -1,   201,
      -1,   202,    -1,   203,    -1,   199,    -1,    33,    -1,     3,
     198,   190,   185,     4,    -1,     3,    29,    98,   208,     3,
     123,     4,    39,   178,    44,   142,     4,    -1,     3,    29,
       1,     4,    -1,     3,    31,    98,   208,     3,   123,     4,
      39,   185,    44,   142,     4,    -1,     3,    31,     1,     4,
      -1,     3,    30,    98,   208,     3,   123,     4,    39,   185,
      44,   157,     4,    -1,     3,    30,     1,     4,    -1,     3,
      32,    98,   208,     3,   123,     4,    48,   161,   204,     4,
      -1,     3,    32,     1,     4,    -1,   204,    44,   147,    -1,
     204,    40,   205,    -1,    -1,   207,    -1,     3,    52,   206,
       4,    -1,   206,   205,    -1,    -1,     3,    49,   185,     4,
      -1,     3,    50,   185,     4,    -1,     3,    51,   185,     4,
      -1,     3,    41,    98,   207,     4,    -1,     3,    41,   207,
       4,    -1,    38,    -1,    10,    -1,    11,    -1,    14,    -1,
      13,    -1,    15,    -1,    16,    -1,    17,    -1,    19,    -1,
      20,    -1,    22,    -1,    21,    -1,    12,    -1,    18,    -1,
      23,    -1,    24,    -1,    25,    -1,    26,    -1,    27,    -1,
      28,    -1,    98,    -1,     3,    34,   125,     4,    -1,     3,
      37,   128,     4,    -1,     3,     7,     3,    61,    98,     4,
       3,    62,    98,     4,   213,     4,    -1,     3,     7,     3,
      61,     1,    -1,   113,   213,    -1,   214,   213,    -1,   215,
     213,    -1,   217,   213,    -1,   195,   213,    -1,   218,   213,
      -1,   219,   213,    -1,    -1,     3,    64,   125,     4,    -1,
       3,    63,   139,     4,    -1,    65,    -1,     3,   216,   178,
       4,    -1,     3,    70,   220,   221,     4,    -1,     3,    70,
       1,     4,    -1,     3,    67,    68,   100,    69,   100,     4,
      -1,     3,    67,    68,   100,     4,    -1,     3,    67,    69,
     100,     4,    -1,    71,    -1,    72,    -1,     3,   222,     4,
      -1,   176,    -1,   174,    -1,    75,    -1,     3,    96,    98,
       4,    -1,     3,    75,     4,    -1,   103,   221,   223,    -1,
     104,   221,   221,    -1,   106,   221,   224,    -1,   105,   221,
     221,    -1,   221,    -1,   221,   223,    -1,   221,    -1,   221,
     224,    -1,   226,   225,    -1,    21,   101,   225,    -1,    21,
     100,   225,    -1,    -1,   229,    86,   227,    -1,   227,    -1,
     228,     5,   229,     6,    -1,   228,    -1,     3,    98,   126,
       4,    -1,   101,    -1,   100,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   240,   240,   241,   242,   246,   252,   259,   260,   261,
     262,   264,   266,   268,   271,   276,   283,   290,   291,   296,
     298,   303,   305,   313,   321,   323,   331,   336,   338,   342,
     344,   351,   364,   373,   382,   394,   396,   402,   411,   421,
     426,   427,   431,   432,   440,   447,   456,   462,   464,   466,
     473,   479,   483,   487,   491,   496,   503,   508,   510,   514,
     516,   520,   525,   527,   529,   532,   536,   542,   543,   545,
     547,   556,   557,   558,   559,   560,   564,   565,   569,   571,
     573,   580,   581,   582,   584,   588,   590,   598,   600,   608,
     613,   618,   621,   628,   629,   633,   635,   637,   641,   645,
     651,   655,   659,   665,   667,   675,   680,   686,   687,   691,
     692,   696,   698,   700,   707,   708,   709,   711,   716,   718,
     720,   722,   724,   729,   735,   741,   746,   747,   751,   752,
     754,   755,   759,   761,   763,   765,   770,   772,   775,   778,
     784,   785,   786,   794,   798,   801,   805,   810,   817,   822,
     827,   832,   837,   839,   841,   843,   845,   850,   852,   854,
     856,   858,   860,   861,   865,   867,   869,   875,   876,   879,
     882,   884,   902,   904,   906,   912,   913,   914,   915,   916,
     928,   930,   944,   946,   948,   950,   954,   959,   961,   963,
     965,   972,   974,   979,   981,   986,   988,   993,   995,   998,
    1000,  1002,  1004,  1006,  1008,  1010,  1012,  1014,  1016,  1021,
    1023,  1027,  1029,  1032,  1035,  1038,  1041,  1054,  1056,  1066,
    1073,  1080,  1085,  1090,  1095,  1097,  1104,  1106,  1113,  1115,
    1122,  1124,  1131,  1132,  1136,  1137,  1138,  1139,  1140,  1144,
    1150,  1159,  1170,  1177,  1188,  1194,  1204,  1210,  1225,  1232,
    1234,  1236,  1240,  1242,  1247,  1250,  1254,  1256,  1258,  1260,
    1265,  1270,  1275,  1276,  1278,  1279,  1281,  1283,  1284,  1285,
    1286,  1287,  1289,  1293,  1302,  1305,  1308,  1310,  1312,  1314,
    1316,  1318,  1324,  1328,  1333,  1345,  1352,  1353,  1354,  1355,
    1356,  1358,  1359,  1360,  1363,  1366,  1369,  1372,  1376,  1378,
    1385,  1388,  1392,  1399,  1400,  1405,  1406,  1407,  1408,  1409,
    1411,  1415,  1416,  1417,  1418,  1422,  1423,  1428,  1429,  1435,
    1438,  1440,  1443,  1447,  1451,  1457,  1461,  1467,  1475,  1476
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "OPEN_BRAC", "CLOSE_BRAC", "OPEN_SQ",
  "CLOSE_SQ", "DEFINE", "PDDLDOMAIN", "REQS", "EQUALITY", "STRIPS", "ADL",
  "NEGATIVE_PRECONDITIONS", "TYPING", "DISJUNCTIVE_PRECONDS", "EXT_PRECS",
  "UNIV_PRECS", "QUANT_PRECS", "COND_EFFS", "FLUENTS", "TIME",
  "DURATIVE_ACTIONS", "DURATION_INEQUALITIES", "CONTINUOUS_EFFECTS",
  "DERIVED_PREDICATES", "TIMED_INITIAL_LITERALS", "PREFERENCES",
  "CONSTRAINTS", "ACTION", "PROCESS", "EVENT", "DURATIVE_ACTION",
  "DERIVED", "CONSTANTS", "PREDS", "FUNCTIONS", "TYPES", "ARGS", "PRE",
  "CONDITION", "PREFERENCE", "START_PRE", "END_PRE", "EFFECTS",
  "INITIAL_EFFECT", "FINAL_EFFECT", "INVARIANT", "DURATION", "AT_START",
  "AT_END", "OVER_ALL", "AND", "OR", "EXISTS", "FORALL", "IMPLY", "NOT",
  "WHEN", "WHENEVER", "EITHER", "PROBLEM", "FORDOMAIN", "INITIALLY",
  "OBJECTS", "GOALS", "EQ", "LENGTH", "SERIAL", "PARALLEL", "METRIC",
  "MINIMIZE", "MAXIMIZE", "HASHT", "DURATION_VAR", "TOTAL_TIME",
  "INCREASE", "DECREASE", "SCALE_UP", "SCALE_DOWN", "ASSIGN", "GREATER",
  "GREATEQ", "LESS", "LESSEQ", "Q", "COLON", "ALWAYS", "SOMETIME",
  "WITHIN", "ATMOSTONCE", "SOMETIMEAFTER", "SOMETIMEBEFORE",
  "ALWAYSWITHIN", "HOLDDURING", "HOLDAFTER", "ISVIOLATED", "BOGUS", "NAME",
  "FUNCTION_SYMBOL", "INTVAL", "FLOATVAL", "AT_TIME", "PLUS", "HYPHEN",
  "DIV", "MUL", "UMINUS", "$accept", "mystartsymbol", "c_domain",
  "c_preamble", "c_domain_name", "c_domain_require_def", "c_reqs",
  "c_pred_decls", "c_pred_decl", "c_new_pred_symbol", "c_pred_symbol",
  "c_init_pred_symbol", "c_func_decls", "c_func_decl", "c_new_func_symbol",
  "c_typed_var_list", "c_var_symbol_list", "c_typed_consts",
  "c_const_symbols", "c_new_const_symbols", "c_typed_types",
  "c_parameter_symbols", "c_declaration_var_symbol", "c_var_symbol",
  "c_const_symbol", "c_new_const_symbol", "c_either_type",
  "c_new_primitive_type", "c_primitive_type", "c_new_primitive_types",
  "c_primitive_types", "c_init_els", "c_timed_initial_literal",
  "c_effects", "c_effect", "c_a_effect", "c_p_effect", "c_p_effects",
  "c_conj_effect", "c_da_effect", "c_da_effects", "c_timed_effect",
  "c_cts_only_timed_effect", "c_da_cts_only_effect",
  "c_da_cts_only_effects", "c_a_effect_da", "c_p_effect_da",
  "c_p_effects_da", "c_f_assign_da", "c_proc_effect", "c_proc_effects",
  "c_f_exp_da", "c_binary_expr_da", "c_duration_constraint", "c_d_op",
  "c_d_value", "c_duration_constraints", "c_neg_simple_effect",
  "c_pos_simple_effect", "c_init_neg_simple_effect",
  "c_init_pos_simple_effect", "c_forall_effect", "c_cond_effect",
  "c_assignment", "c_f_exp", "c_f_exp_t", "c_number", "c_f_head",
  "c_ground_f_head", "c_comparison_op", "c_pre_goal_descriptor",
  "c_pref_con_goal", "c_pref_goal", "c_pref_con_goal_list",
  "c_pref_goal_descriptor", "c_constraint_goal_list", "c_constraint_goal",
  "c_goal_descriptor", "c_goal_list", "c_forall", "c_exists",
  "c_proposition", "c_derived_proposition", "c_init_proposition",
  "c_predicates", "c_functions_def", "c_constraints_def",
  "c_constraints_probdef", "c_structure_defs", "c_structure_def",
  "c_rule_head", "c_derivation_rule", "c_action_def", "c_event_def",
  "c_process_def", "c_durative_action_def", "c_da_def_body", "c_da_gd",
  "c_da_gds", "c_timed_gd", "c_args_head", "c_require_key",
  "c_domain_constants", "c_type_names", "c_problem", "c_problem_body",
  "c_objects", "c_initial_state", "c_goals", "c_goal_spec",
  "c_metric_spec", "c_length_spec", "c_optimization", "c_ground_f_exp",
  "c_binary_ground_f_exp", "c_binary_ground_f_pexps",
  "c_binary_ground_f_mexps", "c_plan", "c_step_t_d", "c_step_d", "c_step",
  "c_float", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315,   316,   317,   318,   319,   320,   321,   322,   323,   324,
     325,   326,   327,   328,   329,   330,   331,   332,   333,   334,
     335,   336,   337,   338,   339,   340,   341,   342,   343,   344,
     345,   346,   347,   348,   349,   350,   351,   352,   353,   354,
     355,   356,   357,   358,   359,   360,   361,   362
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,   108,   109,   109,   109,   110,   110,   111,   111,   111,
     111,   111,   111,   111,   112,   113,   113,   114,   114,   115,
     115,   116,   116,   117,   118,   118,   119,   120,   120,   121,
     121,   122,   123,   123,   123,   124,   124,   125,   125,   125,
     126,   126,   127,   127,   128,   128,   128,   129,   129,   129,
     130,   131,   132,   133,   134,   135,   136,   137,   137,   138,
     138,   139,   139,   139,   139,   139,   140,   141,   141,   141,
     141,   142,   142,   142,   142,   142,   143,   143,   144,   144,
     144,   145,   145,   145,   145,   146,   146,   147,   147,   147,
     147,   147,   147,   148,   148,   149,   149,   149,   149,   149,
     150,   150,   150,   151,   151,   151,   151,   152,   152,   153,
     153,   154,   154,   154,   155,   155,   155,   155,   156,   156,
     156,   156,   156,   157,   157,   157,   158,   158,   159,   159,
     159,   159,   160,   160,   160,   160,   161,   161,   161,   161,
     162,   162,   162,   163,   164,   164,   165,   166,   167,   168,
     169,   170,   171,   171,   171,   171,   171,   172,   172,   172,
     172,   172,   172,   172,   173,   173,   173,   174,   174,   175,
     175,   175,   176,   176,   176,   177,   177,   177,   177,   177,
     178,   178,   179,   179,   179,   179,   179,   180,   180,   180,
     180,   181,   181,   182,   182,   183,   183,   184,   184,   184,
     184,   184,   184,   184,   184,   184,   184,   184,   184,   185,
     185,   185,   185,   185,   185,   185,   185,   186,   186,   187,
     188,   189,   190,   191,   192,   192,   193,   193,   194,   194,
     195,   195,   196,   196,   197,   197,   197,   197,   197,   198,
     199,   200,   200,   201,   201,   202,   202,   203,   203,   204,
     204,   204,   205,   205,   206,   206,   207,   207,   207,   207,
     207,   208,   209,   209,   209,   209,   209,   209,   209,   209,
     209,   209,   209,   209,   209,   209,   209,   209,   209,   209,
     209,   209,   210,   211,   212,   212,   213,   213,   213,   213,
     213,   213,   213,   213,   214,   215,   216,   217,   218,   218,
     219,   219,   219,   220,   220,   221,   221,   221,   221,   221,
     221,   222,   222,   222,   222,   223,   223,   224,   224,   225,
     225,   225,   225,   226,   226,   227,   227,   228,   229,   229
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     1,     1,     5,     4,     2,     2,     2,
       2,     2,     2,     1,     4,     4,     4,     2,     0,     2,
       1,     4,     3,     1,     1,     1,     1,     2,     0,     4,
       3,     1,     4,     4,     1,     3,     0,     4,     4,     1,
       2,     0,     2,     0,     4,     4,     1,     2,     3,     0,
       1,     1,     1,     1,     4,     1,     1,     2,     0,     2,
       0,     6,     2,     2,     2,     0,     4,     2,     2,     2,
       0,     1,     1,     1,     1,     1,     4,     1,     1,     1,
       1,     2,     2,     2,     0,     4,     4,     4,     7,     5,
       5,     1,     1,     2,     0,     4,     4,     5,     5,     3,
       5,     5,     3,     4,     7,     5,     1,     2,     0,     4,
       1,     1,     1,     1,     2,     2,     2,     0,     5,     5,
       5,     5,     5,     5,     5,     4,     2,     0,     1,     2,
       1,     1,     5,     5,     5,     5,     4,     6,     9,     9,
       1,     1,     1,     1,     2,     0,     4,     1,     4,     1,
       7,     5,     5,     5,     5,     5,     5,     4,     5,     5,
       5,     5,     1,     1,     5,     5,     1,     1,     1,     4,
       4,     1,     4,     4,     1,     1,     1,     1,     1,     1,
       1,     1,     4,     5,     4,     7,     1,     4,     5,     4,
       7,     2,     1,     4,     5,     2,     1,     4,     7,     4,
       4,     4,     5,     4,     5,     5,     6,     6,     5,     1,
       4,     4,     4,     5,     7,     7,     5,     2,     1,     1,
       1,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     2,     1,     1,     1,     1,     1,     1,     1,
       5,    12,     4,    12,     4,    12,     4,    11,     4,     3,
       3,     0,     1,     4,     2,     0,     4,     4,     4,     5,
       4,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     4,     4,    12,     5,     2,     2,     2,     2,
       2,     2,     2,     0,     4,     4,     1,     4,     5,     4,
       7,     5,     5,     1,     1,     3,     1,     1,     1,     4,
       3,     3,     3,     3,     3,     1,     2,     1,     2,     2,
       3,     3,     0,     3,     1,     4,     1,     4,     1,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint16 yydefact[] =
{
     322,     0,     0,   329,   328,     0,     2,     3,     4,   322,
     324,   326,     0,     0,    41,   322,   322,     1,     0,   319,
       0,     0,     0,     0,    52,     0,    41,   321,   320,     0,
     323,     0,     0,     6,     0,     0,     0,     0,     0,     0,
      13,   233,   238,   234,   235,   236,   237,     0,     0,   327,
      40,   325,     0,   285,     0,     0,     0,     0,     0,     0,
       0,   239,    43,     0,     0,    58,     0,     5,     7,    10,
      11,    12,     0,   232,     9,     8,    14,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    53,     0,    39,    43,     0,     0,     0,    20,     0,
       0,     0,    46,     0,     0,     0,    16,    15,   262,   263,
     273,   265,   264,   266,   267,   268,   274,   269,   270,   272,
     271,   275,   276,   277,   278,   279,   280,   281,    17,   229,
       0,     0,   219,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,   228,   242,   261,     0,   246,     0,   244,
       0,   248,     0,   282,     0,    42,   225,     0,    23,    36,
     224,    19,   227,     0,   226,    27,   283,    55,     0,    57,
      24,    25,    36,     0,     0,   209,     0,     0,     0,   196,
       0,     0,   167,   168,     0,     0,     0,     0,     0,     0,
       0,    36,    36,    36,    36,    36,     0,    56,    43,    43,
      22,     0,     0,    34,     0,    31,    36,    58,    58,     0,
       0,     0,   220,     0,     0,   179,   175,   176,   177,   178,
      49,     0,     0,     0,   240,     0,   199,   197,   195,   200,
     201,     0,   203,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,    60,    38,    37,    50,    36,    21,     0,
      30,     0,    45,    44,   222,   218,     0,     0,     0,     0,
       0,     0,   171,     0,   162,   163,    36,    36,   293,   202,
     204,   205,     0,     0,   208,     0,     0,     0,     0,     0,
       0,    35,    36,    36,    29,   211,   217,   212,     0,   210,
     221,     0,    47,    49,    49,     0,     0,     0,     0,     0,
       0,     0,     0,   293,   293,     0,   293,   293,   293,   293,
     293,   206,   207,     0,     0,     0,     0,     0,    54,    59,
      33,    32,   213,    51,    48,     0,     0,     0,     0,     0,
       0,   216,     0,     0,     0,    65,    43,   296,     0,     0,
       0,   286,   290,   284,   287,   288,   289,   291,   292,   198,
       0,     0,   180,   181,     0,     0,     0,   251,   170,   169,
       0,   157,     0,     0,     0,     0,     0,     0,     0,     0,
     186,     0,     0,     0,     0,     0,   303,   304,     0,     0,
       0,     0,     0,     0,     0,     0,   145,   142,   141,   140,
       0,     0,   158,   159,   161,   160,   214,   215,   231,     0,
       0,     0,   230,     0,   295,    64,    63,    62,   149,   294,
       0,     0,   299,     0,   308,   174,   307,   306,     0,   297,
       0,     0,     0,     0,    71,    73,    72,    75,    74,   147,
       0,     0,     0,     0,     0,     0,     0,   247,     0,     0,
       0,     0,   192,     0,   186,    36,     0,     0,    26,    65,
      49,   301,     0,   302,     0,     0,    49,    49,     0,     0,
       0,     0,     0,   298,     0,   193,     0,     0,     0,     0,
     241,   127,     0,     0,   245,   243,     0,     0,   136,   144,
       0,     0,   250,   252,     0,   249,    91,    92,     0,   182,
     184,   191,     0,     0,     0,     0,     0,     0,     0,     0,
     310,     0,     0,     0,     0,     0,     0,     0,   305,   194,
       0,     0,     0,    70,    77,    78,    79,    70,    70,    80,
       0,     0,    70,    36,     0,     0,     0,     0,     0,     0,
     143,     0,     0,     0,     0,   255,     0,     0,     0,    94,
       0,     0,     0,     0,     0,     0,     0,     0,   183,     0,
     148,     0,    66,   223,   300,   309,   173,   172,   315,   311,
     312,   314,   317,   313,    86,    84,     0,     0,    85,    67,
      69,    68,   146,     0,     0,   125,   126,     0,   166,     0,
       0,     0,     0,   137,     0,     0,     0,     0,     0,     0,
       0,    99,     0,     0,   110,   113,   111,   112,     0,     0,
       0,     0,     0,     0,     0,     0,     0,    36,     0,     0,
      61,   316,   318,     0,     0,     0,   151,     0,     0,   123,
     124,     0,     0,     0,   260,   256,   257,   258,   253,   254,
     117,     0,     0,     0,     0,     0,    95,    96,    87,    93,
       0,     0,   106,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,   185,     0,    76,    81,
      82,    83,     0,     0,     0,     0,     0,   259,     0,     0,
       0,     0,     0,     0,    89,     0,   108,     0,     0,     0,
       0,    90,     0,   153,    97,   154,    98,   155,   156,   152,
       0,     0,     0,     0,    36,   150,     0,     0,   138,   139,
       0,   109,   116,   114,   115,     0,     0,     0,   128,   130,
     131,     0,     0,     0,     0,   102,     0,     0,     0,     0,
      36,     0,     0,     0,   187,   189,     0,   164,   165,     0,
       0,     0,     0,   129,   119,   120,   121,   122,   118,   103,
     107,     0,     0,     0,     0,    88,   188,     0,     0,     0,
       0,     0,   105,   100,   101,     0,     0,     0,     0,     0,
       0,     0,   190,   132,   133,   135,   134,   104
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     5,     6,    35,    23,   303,    79,    97,    98,   159,
     220,   450,   100,   165,   206,   202,   203,    92,    25,    93,
     101,   260,   247,   324,   292,    94,   198,   169,   199,   102,
     280,   371,   405,   512,   423,   513,   514,   613,   424,   485,
     599,   486,   642,   643,   716,   593,   594,   668,   595,   431,
     524,   707,   708,   357,   390,   529,   435,   515,   516,   406,
     407,   517,   518,   519,   530,   579,   264,   265,   417,   221,
     351,   442,   609,   443,   352,   178,   313,   255,   256,   222,
     223,   175,   104,   408,    37,    38,    39,   304,    40,    41,
      66,    42,    43,    44,    45,    46,   391,   482,   590,   483,
     146,   128,    47,    48,     7,   305,   306,   307,   340,   308,
     309,   310,   378,   558,   462,   559,   563,     8,     9,    10,
      11,    12
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -545
static const yytype_int16 yypact[] =
{
      47,   149,   -53,  -545,  -545,    58,  -545,  -545,  -545,    68,
    -545,   125,   121,   230,   117,    68,    68,  -545,   137,  -545,
     -31,   235,   192,   304,  -545,   237,   117,  -545,  -545,   240,
    -545,   153,    11,  -545,   729,   267,   271,   271,   271,   271,
     279,  -545,  -545,  -545,  -545,  -545,  -545,   271,   271,  -545,
    -545,  -545,   283,  -545,   288,   470,   382,    64,    72,    73,
      89,  -545,   208,   385,   345,  -545,   316,  -545,  -545,  -545,
    -545,  -545,   394,  -545,  -545,  -545,  -545,   320,   323,   429,
     330,   637,   334,   352,   325,   366,   325,   378,   325,   393,
     325,  -545,   414,   285,   208,   431,    90,   455,   407,   458,
     254,   461,   106,   -22,   467,   411,  -545,  -545,  -545,  -545,
    -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,
    -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,
     467,   500,  -545,   467,   467,   167,   467,   467,   467,   167,
     167,   167,   507,  -545,  -545,  -545,   510,  -545,   512,  -545,
     520,  -545,   542,  -545,    97,  -545,  -545,   534,  -545,   464,
    -545,  -545,  -545,   100,  -545,  -545,  -545,  -545,    97,  -545,
    -545,  -545,   464,   338,   555,  -545,   473,   568,   348,  -545,
     569,   571,  -545,  -545,   467,   578,   467,   467,   467,   167,
     467,   464,   464,   464,   464,   464,   523,  -545,   208,   208,
    -545,   491,   587,   488,   589,  -545,   464,  -545,  -545,   590,
     467,   467,  -545,   467,   467,   154,  -545,  -545,  -545,  -545,
    -545,    83,   592,   593,  -545,   600,  -545,  -545,  -545,  -545,
    -545,   605,  -545,   607,   609,   467,   467,   612,   619,   621,
     623,   624,   626,  -545,  -545,  -545,  -545,   464,  -545,    97,
    -545,   628,  -545,  -545,  -545,  -545,   402,   405,   467,   632,
     185,   608,  -545,    83,  -545,  -545,   464,   464,   639,  -545,
    -545,  -545,   640,   641,  -545,   500,   604,   620,   633,   598,
     188,  -545,   464,   464,  -545,  -545,  -545,  -545,   643,  -545,
    -545,   550,  -545,  -545,  -545,    83,    83,    83,    83,   654,
     656,   667,   251,   639,   639,   670,   639,   639,   639,   639,
     639,  -545,  -545,   674,   677,   467,   467,   678,  -545,  -545,
    -545,  -545,  -545,  -545,  -545,   195,   212,    83,    37,    83,
      83,  -545,   467,   467,   395,  -545,   208,  -545,   226,   218,
     677,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,  -545,
     524,   635,  -545,  -545,   644,   646,   462,  -545,  -545,  -545,
     679,  -545,   681,   682,   689,   696,   705,   706,   562,   711,
    -545,   410,   713,   634,   636,   731,  -545,  -545,    40,   733,
     103,   736,   738,   736,   742,   743,  -545,  -545,  -545,  -545,
     662,   232,  -545,  -545,  -545,  -545,  -545,  -545,  -545,   104,
     765,   766,  -545,   157,  -545,  -545,  -545,  -545,  -545,  -545,
     187,   768,  -545,   535,  -545,  -545,  -545,  -545,   769,  -545,
     467,   775,   406,   780,  -545,  -545,  -545,  -545,  -545,  -545,
     -10,   781,   782,   238,   238,   413,   714,  -545,   784,   786,
     500,   791,  -545,   463,  -545,   464,   787,    29,  -545,  -545,
    -545,  -545,   701,  -545,   798,   707,  -545,  -545,    40,    40,
      40,    40,   799,  -545,   800,  -545,   354,   803,   467,   804,
    -545,  -545,    29,    29,  -545,  -545,   723,   724,  -545,  -545,
      83,   193,  -545,  -545,   508,  -545,  -545,  -545,   806,  -545,
    -545,  -545,   807,   715,   808,   370,   167,   496,   213,   810,
    -545,   811,   227,   228,    40,    40,    40,    40,  -545,  -545,
     812,   618,   813,   815,  -545,  -545,  -545,   815,   815,  -545,
     -22,   816,   815,   464,   498,    26,    26,   745,   747,   818,
    -545,   105,   467,   467,   467,  -545,   819,   821,   821,  -545,
     784,   467,    29,    29,    29,    29,    29,   822,  -545,   823,
    -545,   824,  -545,  -545,  -545,  -545,  -545,  -545,    40,  -545,
    -545,  -545,    40,  -545,  -545,  -545,    29,    29,  -545,  -545,
    -545,  -545,  -545,   825,   826,  -545,  -545,   721,  -545,   827,
     828,    83,    83,  -545,   291,   830,   831,   832,   833,   834,
     516,  -545,   625,   835,  -545,  -545,  -545,  -545,   836,   518,
     786,   838,    43,    43,    83,    83,    83,   464,   574,   839,
    -545,  -545,  -545,   522,    83,    83,  -545,   736,    50,  -545,
    -545,   840,   841,   842,  -545,  -545,  -545,  -545,  -545,  -545,
    -545,    29,    29,    29,    29,    29,  -545,  -545,  -545,  -545,
     843,   295,  -545,   844,   645,   845,   846,   847,   848,   849,
     850,   851,   852,   128,   765,   854,  -545,   642,  -545,  -545,
    -545,  -545,   855,    83,   761,   856,   857,  -545,   526,    60,
      60,    60,    60,    60,  -545,   858,  -545,   467,    29,    29,
     860,  -545,    50,  -545,  -545,  -545,  -545,  -545,  -545,  -545,
     786,   500,   861,   529,   464,  -545,   862,   863,  -545,  -545,
     676,  -545,  -545,  -545,  -545,   672,   790,   864,  -545,  -545,
    -545,   865,   866,   867,   868,  -545,   544,   838,    26,    26,
     464,    79,   869,   870,  -545,  -545,   871,  -545,  -545,    60,
      60,    60,    60,  -545,  -545,  -545,  -545,  -545,  -545,  -545,
    -545,   872,   873,   874,   875,  -545,  -545,   823,    60,    60,
      60,    60,  -545,  -545,  -545,   838,   876,   877,   878,   879,
     880,   881,  -545,  -545,  -545,  -545,  -545,  -545
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -545,  -545,  -545,   469,  -545,   744,  -545,   760,  -545,  -545,
     739,  -545,  -545,  -545,  -545,  -170,   647,  -179,   882,   792,
     344,  -232,  -545,  -545,    31,  -545,  -161,  -545,  -151,  -545,
    -545,   438,  -545,  -149,  -377,  -545,  -545,  -545,  -545,  -544,
    -545,  -545,  -545,  -489,  -545,   350,  -545,  -545,   221,   367,
    -545,  -195,  -545,   457,   136,   -20,  -545,  -344,  -329,  -545,
    -545,  -368,  -367,  -436,  -203,  -498,  -130,  -370,  -545,  -545,
     553,  -326,   143,   241,  -545,  -545,   -56,  -103,   685,   -77,
    -545,  -332,  -545,   451,  -545,  -545,  -545,  -545,  -545,   859,
    -545,  -545,  -545,  -545,  -545,  -545,  -545,  -476,  -545,  -505,
     247,  -545,  -545,  -545,  -545,   490,  -545,  -545,  -545,  -545,
    -545,  -545,  -545,  -340,  -545,   340,   339,    63,  -545,   883,
    -545,   885
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -71
static const yytype_int16 yytable[] =
{
      82,   174,   209,   487,   142,   184,   432,   207,   369,   188,
     189,   190,    53,   427,   428,   427,   428,   208,   263,   244,
     245,   238,   239,   240,   241,   242,   586,   177,   580,   577,
     180,   181,   495,   185,   186,   187,   251,   425,   418,   425,
     261,   361,   471,   413,   170,    26,   644,    15,    16,   429,
       1,   429,   426,   261,   426,   639,   640,    26,    17,   236,
     299,   325,   326,   705,   600,    83,   472,   473,     2,     3,
       4,    18,    19,    85,    87,   179,   171,   496,    27,    28,
     623,   231,   261,   233,   234,   235,   261,   237,   282,     2,
      89,   157,   327,   328,   329,   330,   300,   301,   283,   578,
     196,   204,   525,   526,   646,   648,   173,    81,   584,    54,
     258,   259,   320,   321,   629,   414,   578,   491,   504,   505,
     506,   507,   228,   663,   360,   362,   363,   364,   262,   319,
      20,    81,   272,   273,   429,   521,   262,   182,   183,   415,
     182,   183,   262,   182,   183,   706,   722,     3,     4,   262,
     182,   183,   697,   286,   286,   288,    13,   372,   -24,   262,
     182,   183,    84,   487,   487,   560,   561,   562,     3,     4,
      86,    88,   602,   603,   604,   605,   606,   661,   262,   182,
     183,   429,   262,   182,   183,   429,   429,    90,   158,   290,
     429,   451,   318,   596,   596,   197,   614,   615,   205,   358,
      31,   420,   440,   585,   167,   429,   429,    21,   597,   597,
     168,   353,   354,   355,   446,    24,   359,   553,   498,   375,
     742,   743,   562,   447,   502,   503,   691,   740,   741,   365,
     366,   556,   557,    22,   531,    14,   437,   353,    18,   -24,
     662,    49,   532,   533,   534,   535,    51,    14,   416,   427,
     428,    52,   -24,    32,   487,   448,   452,   163,   164,   449,
      55,   669,   670,   671,   672,   673,   761,   182,   183,   659,
     291,    67,   438,   425,    34,   492,   439,   421,   370,   334,
     291,   429,    72,    24,   660,   429,   197,    76,   426,   376,
     377,   401,    77,    24,   373,   374,   675,   291,   291,   710,
     710,   710,   710,   710,   387,    33,    91,    34,   718,   719,
      24,    24,   291,   291,   335,   336,   337,   464,   338,   103,
     388,   339,   389,   105,   703,    24,    24,   106,   416,   416,
     416,   416,   531,   148,   129,   150,   429,   152,   143,   704,
     532,   533,   534,   441,   444,   469,    99,   676,   -28,   -28,
     132,    81,   227,   574,   677,   510,   144,   511,   -70,   710,
     710,   710,   710,   145,   569,   522,   551,   491,   570,   571,
     147,   678,   679,   573,   416,   416,   416,   416,   710,   710,
     710,   710,   149,    80,   488,    81,    95,   370,    96,   154,
     210,   211,   212,   132,   213,   214,   367,   151,   368,   645,
     647,   649,   650,   651,   215,   173,   285,   547,   173,   287,
      96,   645,   647,   403,   404,   664,   356,   478,   153,   216,
     217,   218,   219,    57,    58,    59,    60,    61,   416,   587,
     588,   589,   416,   107,   469,   156,   171,   652,   601,   108,
     109,   110,   111,   112,   113,   114,   115,   116,   117,   118,
     119,   120,   121,   122,   123,   124,   125,   126,   466,   160,
     696,   132,   162,   467,   468,   166,   368,   490,   293,   294,
     173,    78,   170,   176,   -18,   711,   712,   713,   714,   721,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   403,
     552,   430,   575,    81,   171,    68,    69,    70,    71,   536,
     191,   384,   385,   192,   386,   193,    74,    75,   364,   481,
     628,   484,   638,   194,   726,   657,   658,   127,   387,   700,
     701,   655,   368,   725,   748,   749,   750,   751,   200,   709,
     709,   709,   709,   709,   388,   195,   389,   641,   739,   201,
     744,   252,   253,   757,   758,   759,   760,   537,   538,   224,
     539,   621,   622,   132,   680,   380,   540,   541,   -18,   476,
     477,   225,   226,   229,   717,   230,   210,   211,   212,   132,
     213,   214,   232,   243,   542,   543,   544,   545,   546,   246,
     215,   248,   249,   250,   254,   266,   267,   692,   444,   709,
     709,   709,   709,   399,   268,   216,   217,   218,   219,   269,
     454,   270,   130,   271,   400,   653,   274,   132,   709,   709,
     709,   709,   171,   275,   130,   276,   654,   277,   278,   132,
     279,   455,   284,   456,   457,   723,   289,   370,   458,   459,
     460,   461,   302,   314,   311,   312,   317,   322,   323,   133,
     134,   135,   136,   137,   138,   139,   140,   141,   331,   315,
     332,   133,   134,   135,   136,   137,   138,   139,   140,   141,
     565,   333,   316,   132,   343,   467,   468,   630,   349,   381,
     350,   356,   467,   392,   170,   393,   394,   130,   382,   131,
     383,   170,   132,   395,   566,   567,   544,   545,   546,   467,
     396,   631,   632,   633,   634,   635,   293,   294,   170,   397,
     398,   295,   296,   297,   298,   402,   171,   409,   566,   567,
     544,   545,   546,   171,   133,   134,   135,   136,   137,   138,
     139,   140,   141,   467,   410,   412,   411,   419,    55,   422,
     171,   430,   170,   293,   294,   433,   434,   436,   295,   296,
     297,   682,   631,   632,   633,   634,   635,    56,    57,    58,
      59,    60,    61,    62,    63,    64,    65,    36,   368,   445,
     293,   294,   453,   463,   171,   729,   730,   731,   732,   465,
      36,    36,    36,    36,   470,   474,   475,   481,   480,   484,
     493,    36,    36,   341,   342,   489,   344,   345,   346,   347,
     348,   499,   500,   508,   509,   501,   520,   523,   527,   528,
     548,   549,   550,   448,   554,   555,   564,   568,   511,   581,
     572,   582,   583,   591,   592,   607,   608,   618,   610,   616,
     617,   619,   620,   584,   697,   624,   625,   626,   627,   636,
     637,   641,   172,   656,   665,   666,   667,   674,   681,   683,
     684,   685,   686,   687,   688,   689,   690,   694,   161,   695,
     698,   699,   715,   720,   733,   724,   727,   728,   734,   735,
     736,   737,   738,   745,   746,   747,   752,   753,   754,   755,
     762,   763,   764,   765,   766,   767,   155,   497,   598,   702,
     756,   576,   479,   379,   281,   693,   257,   494,   611,    73,
       0,   612,     0,     0,    30,    29,     0,     0,    50
};

static const yytype_int16 yycheck[] =
{
      56,   104,   172,   439,    81,   135,   383,   168,   334,   139,
     140,   141,     1,   381,   381,   383,   383,   168,   221,   198,
     199,   191,   192,   193,   194,   195,   531,   130,   526,     3,
     133,   134,     3,   136,   137,   138,   206,   381,   378,   383,
       3,     4,    52,     3,    66,    14,     3,   100,   101,   381,
       3,   383,   381,     3,   383,   599,   600,    26,     0,   189,
     263,   293,   294,     3,   540,     1,    76,    77,    21,   100,
     101,     3,     9,     1,     1,   131,    98,   447,    15,    16,
     585,   184,     3,   186,   187,   188,     3,   190,   249,    21,
       1,     1,   295,   296,   297,   298,   266,   267,   249,    73,
       3,     1,   472,   473,   602,   603,     3,     3,     3,    98,
     213,   214,   282,   283,   590,    75,    73,   443,   458,   459,
     460,   461,   178,    73,   327,   328,   329,   330,    99,   280,
       5,     3,   235,   236,   466,   467,    99,   100,   101,    99,
     100,   101,    99,   100,   101,    85,   690,   100,   101,    99,
     100,   101,    73,   256,   257,   258,     7,   336,     4,    99,
     100,   101,    98,   599,   600,   505,   506,   507,   100,   101,
      98,    98,   542,   543,   544,   545,   546,   613,    99,   100,
     101,   513,    99,   100,   101,   517,   518,    98,    98,     4,
     522,     4,     4,   537,   538,    98,   566,   567,    98,     4,
       8,    98,    98,    98,    98,   537,   538,    86,   537,   538,
     104,   314,   315,   316,    57,    98,     4,     4,   450,     1,
     718,   719,   562,    66,   456,   457,    98,   716,   717,   332,
     333,     4,     4,     3,    41,    98,     4,   340,     3,    85,
     617,     4,    49,    50,    51,    52,     6,    98,   378,   617,
     617,    98,    98,    61,   690,    98,    69,     3,     4,   102,
       9,   631,   632,   633,   634,   635,   755,   100,   101,   613,
      85,     4,    40,   617,     3,   445,    44,   380,   334,    28,
      85,   613,     3,    98,   613,   617,    98,     4,   617,    71,
      72,   368,     4,    98,    68,    69,     1,    85,    85,   669,
     670,   671,   672,   673,    66,     1,    98,     3,   678,   679,
      98,    98,    85,    85,    63,    64,    65,   420,    67,     3,
      82,    70,    84,     3,   668,    98,    98,     4,   458,   459,
     460,   461,    41,    86,     4,    88,   668,    90,     4,   668,
      49,    50,    51,   399,   400,   422,     1,    52,     3,     4,
      55,     3,     4,   523,    59,     1,     4,     3,     4,   729,
     730,   731,   732,    38,   513,   468,   496,   693,   517,   518,
       4,    76,    77,   522,   504,   505,   506,   507,   748,   749,
     750,   751,     4,     1,   440,     3,     1,   443,     3,   104,
      52,    53,    54,    55,    56,    57,     1,     4,     3,   602,
     603,   604,   605,   606,    66,     3,     4,   484,     3,     4,
       3,   614,   615,     3,     4,   618,     3,     4,     4,    81,
      82,    83,    84,    29,    30,    31,    32,    33,   558,   532,
     533,   534,   562,     4,   511,     4,    98,   607,   541,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    52,     4,
     663,    55,     4,    57,    58,     4,     3,     4,    98,    99,
       3,     1,    66,    62,     4,   670,   671,   672,   673,   682,
      10,    11,    12,    13,    14,    15,    16,    17,    18,    19,
      20,    21,    22,    23,    24,    25,    26,    27,    28,     3,
       4,     3,     4,     3,    98,    36,    37,    38,    39,     1,
       3,    49,    50,     3,    52,     3,    47,    48,   721,     3,
       4,     3,     4,     3,   694,     3,     4,    98,    66,     3,
       4,   608,     3,     4,   729,   730,   731,   732,     4,   669,
     670,   671,   672,   673,    82,     3,    84,     3,     4,    85,
     720,   207,   208,   748,   749,   750,   751,    49,    50,     4,
      52,   581,   582,    55,   641,    41,    58,    59,    98,   433,
     434,    98,     4,     4,   677,     4,    52,    53,    54,    55,
      56,    57,     4,    60,    76,    77,    78,    79,    80,    98,
      66,     4,   104,     4,     4,     3,     3,   653,   654,   729,
     730,   731,   732,    41,     4,    81,    82,    83,    84,     4,
      75,     4,    50,     4,    52,    41,     4,    55,   748,   749,
     750,   751,    98,     4,    50,     4,    52,     4,     4,    55,
       4,    96,     4,    98,    99,   691,     4,   693,   103,   104,
     105,   106,     3,    39,     4,     4,    48,     4,    98,    87,
      88,    89,    90,    91,    92,    93,    94,    95,     4,    39,
       4,    87,    88,    89,    90,    91,    92,    93,    94,    95,
      52,     4,    39,    55,     4,    57,    58,    52,     4,    44,
       3,     3,    57,     4,    66,     4,     4,    50,    44,    52,
      44,    66,    55,     4,    76,    77,    78,    79,    80,    57,
       4,    76,    77,    78,    79,    80,    98,    99,    66,     4,
       4,   103,   104,   105,   106,     4,    98,     4,    76,    77,
      78,    79,    80,    98,    87,    88,    89,    90,    91,    92,
      93,    94,    95,    57,   100,     4,   100,     4,     9,     3,
      98,     3,    66,    98,    99,     3,     3,    85,   103,   104,
     105,   106,    76,    77,    78,    79,    80,    28,    29,    30,
      31,    32,    33,    34,    35,    36,    37,    23,     3,     3,
      98,    99,     4,     4,    98,   103,   104,   105,   106,     4,
      36,    37,    38,    39,     4,     4,     4,     3,    74,     3,
       3,    47,    48,   303,   304,     4,   306,   307,   308,   309,
     310,   100,     4,     4,     4,    98,     3,     3,    85,    85,
       4,     4,     4,    98,     4,     4,     4,     4,     3,    74,
       4,    74,     4,     4,     3,     3,     3,   106,     4,     4,
       4,     4,     4,     3,    73,     4,     4,     4,     4,     4,
       4,     3,   103,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     3,    98,     4,
       4,     4,     4,     3,    74,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,    94,   449,   538,   668,
     747,   524,   435,   340,   247,   654,   211,   446,   558,    40,
      -1,   562,    -1,    -1,    21,    20,    -1,    -1,    26
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,    21,   100,   101,   109,   110,   212,   225,   226,
     227,   228,   229,     7,    98,   100,   101,     0,     3,   225,
       5,    86,     3,   112,    98,   126,   132,   225,   225,   229,
     227,     8,    61,     1,     3,   111,   113,   192,   193,   194,
     196,   197,   199,   200,   201,   202,   203,   210,   211,     4,
     126,     6,    98,     1,    98,     9,    28,    29,    30,    31,
      32,    33,    34,    35,    36,    37,   198,     4,   111,   111,
     111,   111,     3,   197,   111,   111,     4,     4,     1,   114,
       1,     3,   184,     1,    98,     1,    98,     1,    98,     1,
      98,    98,   125,   127,   133,     1,     3,   115,   116,     1,
     120,   128,   137,     3,   190,     3,     4,     4,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28,    98,   209,     4,
      50,    52,    55,    87,    88,    89,    90,    91,    92,    93,
      94,    95,   187,     4,     4,    38,   208,     4,   208,     4,
     208,     4,   208,     4,   104,   127,     4,     1,    98,   117,
       4,   115,     4,     3,     4,   121,     4,    98,   104,   135,
      66,    98,   118,     3,   185,   189,    62,   185,   183,   184,
     185,   185,   100,   101,   174,   185,   185,   185,   174,   174,
     174,     3,     3,     3,     3,     3,     3,    98,   134,   136,
       4,    85,   123,   124,     1,    98,   122,   134,   136,   123,
      52,    53,    54,    56,    57,    66,    81,    82,    83,    84,
     118,   177,   187,   188,     4,    98,     4,     4,   184,     4,
       4,   185,     4,   185,   185,   185,   174,   185,   123,   123,
     123,   123,   123,    60,   125,   125,    98,   130,     4,   104,
       4,   123,   128,   128,     4,   185,   186,   186,   185,   185,
     129,     3,    99,   172,   174,   175,     3,     3,     4,     4,
       4,     4,   185,   185,     4,     4,     4,     4,     4,     4,
     138,   124,   134,   136,     4,     4,   185,     4,   185,     4,
       4,    85,   132,    98,    99,   103,   104,   105,   106,   172,
     123,   123,     3,   113,   195,   213,   214,   215,   217,   218,
     219,     4,     4,   184,    39,    39,    39,    48,     4,   136,
     123,   123,     4,    98,   131,   129,   129,   172,   172,   172,
     172,     4,     4,     4,    28,    63,    64,    65,    67,    70,
     216,   213,   213,     4,   213,   213,   213,   213,   213,     4,
       3,   178,   182,   185,   185,   185,     3,   161,     4,     4,
     172,     4,   172,   172,   172,   185,   185,     1,     3,   179,
     184,   139,   125,    68,    69,     1,    71,    72,   220,   178,
      41,    44,    44,    44,    49,    50,    52,    66,    82,    84,
     162,   204,     4,     4,     4,     4,     4,     4,     4,    41,
      52,   187,     4,     3,     4,   140,   167,   168,   191,     4,
     100,   100,     4,     3,    75,    99,   174,   176,   221,     4,
      98,   185,     3,   142,   146,   165,   166,   169,   170,   189,
       3,   157,   142,     3,     3,   164,    85,     4,    40,    44,
      98,   184,   179,   181,   184,     3,    57,    66,    98,   102,
     119,     4,    69,     4,    75,    96,    98,    99,   103,   104,
     105,   106,   222,     4,   185,     4,    52,    57,    58,   187,
       4,    52,    76,    77,     4,     4,   162,   162,     4,   161,
      74,     3,   205,   207,     3,   147,   149,   171,   184,     4,
       4,   179,   123,     3,   191,     3,   175,   139,   129,   100,
       4,    98,   129,   129,   221,   221,   221,   221,     4,     4,
       1,     3,   141,   143,   144,   165,   166,   169,   170,   171,
       3,   189,   185,     3,   158,   175,   175,    85,    85,   163,
     172,    41,    49,    50,    51,    52,     1,    49,    50,    52,
      58,    59,    76,    77,    78,    79,    80,   187,     4,     4,
       4,   174,     4,     4,     4,     4,     4,     4,   221,   223,
     221,   221,   221,   224,     4,    52,    76,    77,     4,   141,
     141,   141,     4,   141,   123,     4,   157,     3,    73,   173,
     173,    74,    74,     4,     3,    98,   207,   185,   185,   185,
     206,     4,     3,   153,   154,   156,   165,   166,   153,   148,
     205,   185,   175,   175,   175,   175,   175,     3,     3,   180,
       4,   223,   224,   145,   175,   175,     4,     4,   106,     4,
       4,   163,   163,   207,     4,     4,     4,     4,     4,   205,
      52,    76,    77,    78,    79,    80,     4,     4,     4,   147,
     147,     3,   150,   151,     3,   172,   173,   172,   173,   172,
     172,   172,   123,    41,    52,   187,     4,     3,     4,   165,
     166,   171,   142,    73,   172,     4,     4,     4,   155,   175,
     175,   175,   175,   175,     4,     1,    52,    59,    76,    77,
     187,     4,   106,     4,     4,     4,     4,     4,     4,     4,
       4,    98,   184,   181,     3,     4,   172,    73,     4,     4,
       3,     4,   156,   165,   166,     3,    85,   159,   160,   174,
     175,   159,   159,   159,   159,     4,   152,   185,   175,   175,
       3,   172,   147,   184,     4,     4,   123,     4,     4,   103,
     104,   105,   106,    74,     4,     4,     4,     4,     4,     4,
     151,   151,   173,   173,   123,     4,     4,     4,   159,   159,
     159,   159,     4,     4,     4,     4,   180,   159,   159,   159,
     159,   151,     4,     4,     4,     4,     4,     4
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *bottom, yytype_int16 *top)
#else
static void
yy_stack_print (bottom, top)
    yytype_int16 *bottom;
    yytype_int16 *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      fprintf (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      fprintf (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The look-ahead symbol.  */
int yychar;

/* The semantic value of the look-ahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  int yystate;
  int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Look-ahead token as an internal (translated) token number.  */
  int yytoken = 0;
#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  yytype_int16 yyssa[YYINITDEPTH];
  yytype_int16 *yyss = yyssa;
  yytype_int16 *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  YYSTYPE *yyvsp;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;


	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),

		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss);
	YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;


      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     look-ahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to look-ahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a look-ahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid look-ahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the look-ahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 240 "../pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_domain); current_analysis->the_domain= (yyvsp[(1) - (1)].t_domain);;}
    break;

  case 3:
#line 241 "../pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_problem); current_analysis->the_problem= (yyvsp[(1) - (1)].t_problem);;}
    break;

  case 4:
#line 242 "../pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_plan); ;}
    break;

  case 5:
#line 247 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(4) - (5)].t_domain); (yyval.t_domain)->name= (yyvsp[(3) - (5)].cp);delete [] (yyvsp[(3) - (5)].cp);
	if (types_used && !types_defined) {
		yyerrok; log_error(E_FATAL,"Syntax error in domain - no :types section, but types used in definitions."); 
	}
	;}
    break;

  case 6:
#line 253 "../pddl+.yacc"
    {yyerrok; (yyval.t_domain)=static_cast<domain*>(NULL);
       	log_error(E_FATAL,"Syntax error in domain"); ;}
    break;

  case 7:
#line 259 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 8:
#line 260 "../pddl+.yacc"
    {types_defined = true; (yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->types= (yyvsp[(1) - (2)].t_type_list);;}
    break;

  case 9:
#line 261 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->constants= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 10:
#line 262 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->predicates= (yyvsp[(1) - (2)].t_pred_decl_list); ;}
    break;

  case 11:
#line 264 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->functions= (yyvsp[(1) - (2)].t_func_decl_list); ;}
    break;

  case 12:
#line 266 "../pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain);
   										(yyval.t_domain)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 13:
#line 268 "../pddl+.yacc"
    {(yyval.t_domain)= new domain((yyvsp[(1) - (1)].t_structure_store)); ;}
    break;

  case 14:
#line 271 "../pddl+.yacc"
    {(yyval.cp)=(yyvsp[(3) - (4)].cp);;}
    break;

  case 15:
#line 277 "../pddl+.yacc"
    {
	// Stash in analysis object --- we need to refer to it during parse
	//   but domain object is not created yet,
	current_analysis->req |= (yyvsp[(3) - (4)].t_pddl_req_flag);
	(yyval.t_pddl_req_flag)=(yyvsp[(3) - (4)].t_pddl_req_flag);
    ;}
    break;

  case 16:
#line 284 "../pddl+.yacc"
    {yyerrok; 
       log_error(E_FATAL,"Syntax error in requirements declaration.");
       (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 17:
#line 290 "../pddl+.yacc"
    { (yyval.t_pddl_req_flag)= (yyvsp[(1) - (2)].t_pddl_req_flag) | (yyvsp[(2) - (2)].t_pddl_req_flag); ;}
    break;

  case 18:
#line 291 "../pddl+.yacc"
    { (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 19:
#line 297 "../pddl+.yacc"
    {(yyval.t_pred_decl_list)=(yyvsp[(2) - (2)].t_pred_decl_list); (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (2)].t_pred_decl));;}
    break;

  case 20:
#line 299 "../pddl+.yacc"
    {  (yyval.t_pred_decl_list)=new pred_decl_list;
           (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (1)].t_pred_decl)); ;}
    break;

  case 21:
#line 304 "../pddl+.yacc"
    {(yyval.t_pred_decl)= new pred_decl((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 22:
#line 306 "../pddl+.yacc"
    {yyerrok; 
        // hope someone makes this error someday
        log_error(E_FATAL,"Syntax error in predicate declaration.");
	(yyval.t_pred_decl)= NULL; ;}
    break;

  case 23:
#line 314 "../pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           				current_analysis->buildPredTab());
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 24:
#line 321 "../pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_ref("="); 
	      requires(E_EQUALITY); ;}
    break;

  case 25:
#line 323 "../pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 26:
#line 331 "../pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 27:
#line 337 "../pddl+.yacc"
    {(yyval.t_func_decl_list)=(yyvsp[(1) - (2)].t_func_decl_list); (yyval.t_func_decl_list)->push_back((yyvsp[(2) - (2)].t_func_decl));;}
    break;

  case 28:
#line 338 "../pddl+.yacc"
    { (yyval.t_func_decl_list)=new func_decl_list; ;}
    break;

  case 29:
#line 343 "../pddl+.yacc"
    {(yyval.t_func_decl)= new func_decl((yyvsp[(2) - (4)].t_func_symbol),(yyvsp[(3) - (4)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 30:
#line 345 "../pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in functor declaration.");
	 (yyval.t_func_decl)= NULL; ;}
    break;

  case 31:
#line 352 "../pddl+.yacc"
    { (yyval.t_func_symbol)=current_analysis->func_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           		current_analysis->buildFuncTab()); 
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 32:
#line 365 "../pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 33:
#line 374 "../pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));    /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 34:
#line 383 "../pddl+.yacc"
    {
       (yyval.t_var_symbol_list)= (yyvsp[(1) - (1)].t_var_symbol_list);
   ;}
    break;

  case 35:
#line 395 "../pddl+.yacc"
    {(yyval.t_var_symbol_list)=(yyvsp[(3) - (3)].t_var_symbol_list); (yyvsp[(3) - (3)].t_var_symbol_list)->push_front((yyvsp[(2) - (3)].t_var_symbol)); ;}
    break;

  case 36:
#line 396 "../pddl+.yacc"
    {(yyval.t_var_symbol_list)= new var_symbol_list; ;}
    break;

  case 37:
#line 403 "../pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list)); /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_const_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 38:
#line 412 "../pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list));
      delete (yyvsp[(4) - (4)].t_const_symbol_list);
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 39:
#line 421 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)= (yyvsp[(1) - (1)].t_const_symbol_list);;}
    break;

  case 40:
#line 426 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 41:
#line 427 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 42:
#line 431 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 43:
#line 432 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 44:
#line 441 "../pddl+.yacc"
    {  
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
       (yyval.t_type_list)->splice((yyval.t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list)); /* Join lists */ 
       delete (yyvsp[(4) - (4)].t_type_list);                   /* Delete (now empty) list */
   ;}
    break;

  case 45:
#line 448 "../pddl+.yacc"
    {  
   // This parse needs to be excluded, we think (DPL&MF: 6/9/01)
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
       (yyval.t_type_list)->splice((yyvsp[(1) - (4)].t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list));
       delete (yyvsp[(4) - (4)].t_type_list);
   ;}
    break;

  case 46:
#line 457 "../pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(1) - (1)].t_type_list); ;}
    break;

  case 47:
#line 463 "../pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (2)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(2) - (2)].t_const_symbol)); ;}
    break;

  case 48:
#line 465 "../pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (3)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(3) - (3)].t_var_symbol)); ;}
    break;

  case 49:
#line 466 "../pddl+.yacc"
    {(yyval.t_parameter_symbol_list)= new parameter_symbol_list;;}
    break;

  case 50:
#line 473 "../pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.top()->symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 51:
#line 479 "../pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 52:
#line 483 "../pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 53:
#line 487 "../pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 54:
#line 492 "../pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(3) - (4)].t_type_list); ;}
    break;

  case 55:
#line 497 "../pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 56:
#line 504 "../pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 57:
#line 509 "../pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 58:
#line 510 "../pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 59:
#line 515 "../pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 60:
#line 516 "../pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 61:
#line 521 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (6)].t_effect_lists);
	  (yyval.t_effect_lists)->assign_effects.push_back(new assignment((yyvsp[(4) - (6)].t_func_term),E_ASSIGN,(yyvsp[(5) - (6)].t_num_expression)));  
          requires(E_FLUENTS); 
	;}
    break;

  case 62:
#line 526 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 63:
#line 528 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 64:
#line 530 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(2) - (2)].t_timed_effect)); ;}
    break;

  case 65:
#line 532 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;;}
    break;

  case 66:
#line 537 "../pddl+.yacc"
    { requires(E_TIMED_INITIAL_LITERALS); 
   		(yyval.t_timed_effect)=new timed_initial_literal((yyvsp[(3) - (4)].t_effect_lists),(yyvsp[(2) - (4)].fval));;}
    break;

  case 67:
#line 542 "../pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->append_effects((yyvsp[(1) - (2)].t_effect_lists)); delete (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 68:
#line 543 "../pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (2)].t_cond_effect)); 
                                      requires(E_COND_EFFS);;}
    break;

  case 69:
#line 545 "../pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (2)].t_forall_effect));
                                      requires(E_COND_EFFS);;}
    break;

  case 70:
#line 547 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists(); ;}
    break;

  case 71:
#line 556 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 72:
#line 557 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 73:
#line 558 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 74:
#line 559 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (1)].t_cond_effect));;}
    break;

  case 75:
#line 560 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (1)].t_forall_effect));;}
    break;

  case 76:
#line 564 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 77:
#line 565 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 78:
#line 570 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 79:
#line 572 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 80:
#line 574 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_FLUENTS);;}
    break;

  case 81:
#line 580 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 82:
#line 581 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 83:
#line 582 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_FLUENTS); ;}
    break;

  case 84:
#line 584 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 85:
#line 589 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 86:
#line 591 "../pddl+.yacc"
    {yyerrok; (yyval.t_effect_lists)=NULL;
	 log_error(E_FATAL,"Syntax error in (and ...)");
	;}
    break;

  case 87:
#line 599 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 88:
#line 604 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);;}
    break;

  case 89:
#line 609 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 90:
#line 614 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_assign_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 91:
#line 619 "../pddl+.yacc"
    { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(1) - (1)].t_timed_effect)); ;}
    break;

  case 92:
#line 622 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
          requires(E_FLUENTS); ;}
    break;

  case 93:
#line 628 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 94:
#line 629 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 95:
#line 634 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_START);;}
    break;

  case 96:
#line 636 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_END);;}
    break;

  case 97:
#line 638 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 98:
#line 642 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 99:
#line 646 "../pddl+.yacc"
    {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in timed effect"); ;}
    break;

  case 100:
#line 652 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 101:
#line 656 "../pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 102:
#line 660 "../pddl+.yacc"
    {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in conditional continuous effect"); ;}
    break;

  case 103:
#line 666 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 104:
#line 671 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);;}
    break;

  case 105:
#line 676 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_assign_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 106:
#line 681 "../pddl+.yacc"
    { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(1) - (1)].t_timed_effect)); ;}
    break;

  case 107:
#line 686 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 108:
#line 687 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 109:
#line 691 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 110:
#line 692 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 111:
#line 697 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 112:
#line 699 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 113:
#line 701 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_FLUENTS);;}
    break;

  case 114:
#line 707 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 115:
#line 708 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 116:
#line 709 "../pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_FLUENTS); ;}
    break;

  case 117:
#line 711 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 118:
#line 717 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 119:
#line 719 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 120:
#line 721 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 121:
#line 723 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 122:
#line 725 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 123:
#line 730 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 124:
#line 736 "../pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 125:
#line 742 "../pddl+.yacc"
    {(yyval.t_effect_lists) = (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 126:
#line 746 "../pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 127:
#line 747 "../pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 128:
#line 751 "../pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 129:
#line 752 "../pddl+.yacc"
    {(yyval.t_expression)= new special_val_expr(E_DURATION_VAR);
                    requires( E_DURATION_INEQUALITIES );;}
    break;

  case 130:
#line 754 "../pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 131:
#line 755 "../pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); ;}
    break;

  case 132:
#line 760 "../pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 133:
#line 762 "../pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 134:
#line 764 "../pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 135:
#line 766 "../pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 136:
#line 771 "../pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 137:
#line 773 "../pddl+.yacc"
    { (yyval.t_goal)= new timed_goal(new comparison((yyvsp[(2) - (6)].t_comparison_op),
        			new special_val_expr(E_DURATION_VAR),(yyvsp[(5) - (6)].t_expression)),E_AT_START); ;}
    break;

  case 138:
#line 776 "../pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_START);;}
    break;

  case 139:
#line 779 "../pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_END);;}
    break;

  case 140:
#line 784 "../pddl+.yacc"
    {(yyval.t_comparison_op)= E_LESSEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 141:
#line 785 "../pddl+.yacc"
    {(yyval.t_comparison_op)= E_GREATEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 142:
#line 786 "../pddl+.yacc"
    {(yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 143:
#line 794 "../pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression); ;}
    break;

  case 144:
#line 799 "../pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 145:
#line 801 "../pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 146:
#line 806 "../pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 147:
#line 811 "../pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 148:
#line 818 "../pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 149:
#line 823 "../pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 150:
#line 828 "../pddl+.yacc"
    { (yyval.t_forall_effect)= new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop());;}
    break;

  case 151:
#line 833 "../pddl+.yacc"
    { (yyval.t_cond_effect)= new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)); ;}
    break;

  case 152:
#line 838 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 153:
#line 840 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 154:
#line 842 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 155:
#line 844 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 156:
#line 846 "../pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 157:
#line 851 "../pddl+.yacc"
    { (yyval.t_expression)= new uminus_expression((yyvsp[(3) - (4)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 158:
#line 853 "../pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 159:
#line 855 "../pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 160:
#line 857 "../pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 161:
#line 859 "../pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 162:
#line 860 "../pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 163:
#line 861 "../pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); requires(E_FLUENTS); ;}
    break;

  case 164:
#line 866 "../pddl+.yacc"
    { (yyval.t_expression)= new mul_expression(new special_val_expr(E_HASHT),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 165:
#line 868 "../pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression), new special_val_expr(E_HASHT)); ;}
    break;

  case 166:
#line 870 "../pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_HASHT); ;}
    break;

  case 167:
#line 875 "../pddl+.yacc"
    { (yyval.t_num_expression)=new int_expression((yyvsp[(1) - (1)].ival));   ;}
    break;

  case 168:
#line 876 "../pddl+.yacc"
    { (yyval.t_num_expression)=new float_expression((yyvsp[(1) - (1)].fval)); ;}
    break;

  case 169:
#line 880 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 170:
#line 883 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 171:
#line 885 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 172:
#line 903 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 173:
#line 905 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 174:
#line 907 "../pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 175:
#line 912 "../pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATER; ;}
    break;

  case 176:
#line 913 "../pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATEQ; ;}
    break;

  case 177:
#line 914 "../pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESS; ;}
    break;

  case 178:
#line 915 "../pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESSEQ; ;}
    break;

  case 179:
#line 916 "../pddl+.yacc"
    { (yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 180:
#line 929 "../pddl+.yacc"
    {(yyval.t_goal)= (yyvsp[(1) - (1)].t_goal);;}
    break;

  case 181:
#line 931 "../pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(1) - (1)].t_goal);;}
    break;

  case 182:
#line 945 "../pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 183:
#line 947 "../pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 184:
#line 949 "../pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 185:
#line 952 "../pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 186:
#line 955 "../pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(1) - (1)].t_con_goal);;}
    break;

  case 187:
#line 960 "../pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 188:
#line 962 "../pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 189:
#line 964 "../pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 190:
#line 967 "../pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 191:
#line 973 "../pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 192:
#line 975 "../pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_con_goal));;}
    break;

  case 193:
#line 980 "../pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (4)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 194:
#line 982 "../pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 195:
#line 987 "../pddl+.yacc"
    {(yyval.t_goal_list) = (yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 196:
#line 989 "../pddl+.yacc"
    {(yyval.t_goal_list) = new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_con_goal));;}
    break;

  case 197:
#line 994 "../pddl+.yacc"
    {(yyval.t_con_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 198:
#line 996 "../pddl+.yacc"
    {(yyval.t_con_goal) = new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);;}
    break;

  case 199:
#line 999 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATEND,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 200:
#line 1001 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYS,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 201:
#line 1003 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIME,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 202:
#line 1005 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_WITHIN,(yyvsp[(4) - (5)].t_goal),NULL,(yyvsp[(3) - (5)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 203:
#line 1007 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATMOSTONCE,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 204:
#line 1009 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEAFTER,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 205:
#line 1011 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEBEFORE,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 206:
#line 1013 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYSWITHIN,(yyvsp[(5) - (6)].t_goal),(yyvsp[(4) - (6)].t_goal),(yyvsp[(3) - (6)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (6)].t_num_expression);;}
    break;

  case 207:
#line 1015 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDDURING,(yyvsp[(5) - (6)].t_goal),NULL,(yyvsp[(4) - (6)].t_num_expression)->double_value(),(yyvsp[(3) - (6)].t_num_expression)->double_value());delete (yyvsp[(3) - (6)].t_num_expression);delete (yyvsp[(4) - (6)].t_num_expression);;}
    break;

  case 208:
#line 1017 "../pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDAFTER,(yyvsp[(4) - (5)].t_goal),NULL,0.0,(yyvsp[(3) - (5)].t_num_expression)->double_value());delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 209:
#line 1022 "../pddl+.yacc"
    {(yyval.t_goal)= new simple_goal((yyvsp[(1) - (1)].t_proposition),E_POS);;}
    break;

  case 210:
#line 1024 "../pddl+.yacc"
    {(yyval.t_goal)= new neg_goal((yyvsp[(3) - (4)].t_goal));simple_goal * s = dynamic_cast<simple_goal *>((yyvsp[(3) - (4)].t_goal));
       if(s && s->getProp()->head->getName()=="=") {requires(E_EQUALITY);} 
       else{requires(E_NEGATIVE_PRECONDITIONS);};;}
    break;

  case 211:
#line 1028 "../pddl+.yacc"
    {(yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 212:
#line 1030 "../pddl+.yacc"
    {(yyval.t_goal)= new disj_goal((yyvsp[(3) - (4)].t_goal_list));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 213:
#line 1033 "../pddl+.yacc"
    {(yyval.t_goal)= new imply_goal((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_goal));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 214:
#line 1037 "../pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal((yyvsp[(2) - (7)].t_quantifier),(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 215:
#line 1040 "../pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal((yyvsp[(2) - (7)].t_quantifier),(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 216:
#line 1042 "../pddl+.yacc"
    {(yyval.t_goal)= new comparison((yyvsp[(2) - (5)].t_comparison_op),(yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); 
        requires(E_FLUENTS);;}
    break;

  case 217:
#line 1055 "../pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal));;}
    break;

  case 218:
#line 1057 "../pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_goal));;}
    break;

  case 219:
#line 1067 "../pddl+.yacc"
    {(yyval.t_quantifier)=E_FORALL; 
        current_analysis->var_tab_stack.push(
        		current_analysis->buildForallTab());;}
    break;

  case 220:
#line 1074 "../pddl+.yacc"
    {(yyval.t_quantifier)=E_EXISTS;
        current_analysis->var_tab_stack.push(
        	current_analysis->buildExistsTab());;}
    break;

  case 221:
#line 1081 "../pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 222:
#line 1086 "../pddl+.yacc"
    {(yyval.t_proposition) = new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list));;}
    break;

  case 223:
#line 1091 "../pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 224:
#line 1096 "../pddl+.yacc"
    {(yyval.t_pred_decl_list)= (yyvsp[(3) - (4)].t_pred_decl_list);;}
    break;

  case 225:
#line 1098 "../pddl+.yacc"
    {yyerrok; (yyval.t_pred_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:predicates ...)");
	;}
    break;

  case 226:
#line 1105 "../pddl+.yacc"
    {(yyval.t_func_decl_list)= (yyvsp[(3) - (4)].t_func_decl_list);;}
    break;

  case 227:
#line 1107 "../pddl+.yacc"
    {yyerrok; (yyval.t_func_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:functions ...)");
	;}
    break;

  case 228:
#line 1114 "../pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 229:
#line 1116 "../pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 230:
#line 1123 "../pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 231:
#line 1125 "../pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 232:
#line 1131 "../pddl+.yacc"
    { (yyval.t_structure_store)=(yyvsp[(1) - (2)].t_structure_store); (yyval.t_structure_store)->push_back((yyvsp[(2) - (2)].t_structure_def)); ;}
    break;

  case 233:
#line 1132 "../pddl+.yacc"
    { (yyval.t_structure_store)= new structure_store; (yyval.t_structure_store)->push_back((yyvsp[(1) - (1)].t_structure_def)); ;}
    break;

  case 234:
#line 1136 "../pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_action_def); ;}
    break;

  case 235:
#line 1137 "../pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_event_def); requires(E_TIME); ;}
    break;

  case 236:
#line 1138 "../pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_process_def); requires(E_TIME); ;}
    break;

  case 237:
#line 1139 "../pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_durative_action_def); requires(E_DURATIVE_ACTIONS); ;}
    break;

  case 238:
#line 1140 "../pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_derivation_rule); requires(E_DERIVED_PREDICATES);;}
    break;

  case 239:
#line 1144 "../pddl+.yacc"
    {(yyval.t_dummy)= 0; 
    	current_analysis->var_tab_stack.push(
    					current_analysis->buildRuleTab());;}
    break;

  case 240:
#line 1155 "../pddl+.yacc"
    {(yyval.t_derivation_rule) = new derivation_rule((yyvsp[(3) - (5)].t_proposition),(yyvsp[(4) - (5)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 241:
#line 1167 "../pddl+.yacc"
    { (yyval.t_action_def)= current_analysis->buildAction(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
			(yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
			current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp); ;}
    break;

  case 242:
#line 1171 "../pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in action declaration.");
	 (yyval.t_action_def)= NULL; ;}
    break;

  case 243:
#line 1184 "../pddl+.yacc"
    {(yyval.t_event_def)= current_analysis->buildEvent(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		   (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
		   current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 244:
#line 1189 "../pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in event declaration.");
	 (yyval.t_event_def)= NULL; ;}
    break;

  case 245:
#line 1201 "../pddl+.yacc"
    {(yyval.t_process_def)= current_analysis->buildProcess(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		     (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
                     current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 246:
#line 1205 "../pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in process declaration.");
	 (yyval.t_process_def)= NULL; ;}
    break;

  case 247:
#line 1217 "../pddl+.yacc"
    { (yyval.t_durative_action_def)= (yyvsp[(10) - (11)].t_durative_action_def);
      (yyval.t_durative_action_def)->name= current_analysis->op_tab.symbol_put((yyvsp[(3) - (11)].cp));
      (yyval.t_durative_action_def)->symtab= current_analysis->var_tab_stack.pop();
      (yyval.t_durative_action_def)->parameters= (yyvsp[(6) - (11)].t_var_symbol_list);
      (yyval.t_durative_action_def)->dur_constraint= (yyvsp[(9) - (11)].t_goal); 
      delete [] (yyvsp[(3) - (11)].cp);
    ;}
    break;

  case 248:
#line 1226 "../pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in durative-action declaration.");
	 (yyval.t_durative_action_def)= NULL; ;}
    break;

  case 249:
#line 1233 "../pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->effects=(yyvsp[(3) - (3)].t_effect_lists);;}
    break;

  case 250:
#line 1235 "../pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->precondition=(yyvsp[(3) - (3)].t_goal);;}
    break;

  case 251:
#line 1236 "../pddl+.yacc"
    {(yyval.t_durative_action_def)= current_analysis->buildDurativeAction();;}
    break;

  case 252:
#line 1241 "../pddl+.yacc"
    { (yyval.t_goal)=(yyvsp[(1) - (1)].t_goal); ;}
    break;

  case 253:
#line 1243 "../pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 254:
#line 1248 "../pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 255:
#line 1250 "../pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 256:
#line 1255 "../pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_START);;}
    break;

  case 257:
#line 1257 "../pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_END);;}
    break;

  case 258:
#line 1259 "../pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_OVER_ALL);;}
    break;

  case 259:
#line 1261 "../pddl+.yacc"
    {timed_goal * tg = dynamic_cast<timed_goal *>((yyvsp[(4) - (5)].t_goal));
		(yyval.t_goal) = new timed_goal(new preference((yyvsp[(3) - (5)].cp),tg->clearGoal()),tg->getTime());
			delete tg;
			requires(E_PREFERENCES);;}
    break;

  case 260:
#line 1266 "../pddl+.yacc"
    {(yyval.t_goal) = new preference((yyvsp[(3) - (4)].t_goal));requires(E_PREFERENCES);;}
    break;

  case 261:
#line 1270 "../pddl+.yacc"
    {(yyval.t_dummy)= 0; current_analysis->var_tab_stack.push(
    				current_analysis->buildOpTab());;}
    break;

  case 262:
#line 1275 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EQUALITY;;}
    break;

  case 263:
#line 1276 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS;;}
    break;

  case 264:
#line 1278 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TYPING;;}
    break;

  case 265:
#line 1280 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_NEGATIVE_PRECONDITIONS;;}
    break;

  case 266:
#line 1282 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DISJUNCTIVE_PRECONDS;;}
    break;

  case 267:
#line 1283 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS;;}
    break;

  case 268:
#line 1284 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_UNIV_PRECS;;}
    break;

  case 269:
#line 1285 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_COND_EFFS;;}
    break;

  case 270:
#line 1286 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_FLUENTS;;}
    break;

  case 271:
#line 1288 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATIVE_ACTIONS;;}
    break;

  case 272:
#line 1289 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TIME |
                      E_FLUENTS |
                      E_DURATIVE_ACTIONS; ;}
    break;

  case 273:
#line 1293 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS |
		      E_TYPING | 
		      E_NEGATIVE_PRECONDITIONS |
		      E_DISJUNCTIVE_PRECONDS |
		      E_EQUALITY |
		      E_EXT_PRECS |
		      E_UNIV_PRECS |
		      E_COND_EFFS;;}
    break;

  case 274:
#line 1302 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS |
		      E_UNIV_PRECS;;}
    break;

  case 275:
#line 1306 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATION_INEQUALITIES;;}
    break;

  case 276:
#line 1309 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_CONTINUOUS_EFFECTS;;}
    break;

  case 277:
#line 1311 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_DERIVED_PREDICATES;;}
    break;

  case 278:
#line 1313 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_TIMED_INITIAL_LITERALS;;}
    break;

  case 279:
#line 1315 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_PREFERENCES;;}
    break;

  case 280:
#line 1317 "../pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_CONSTRAINTS;;}
    break;

  case 281:
#line 1319 "../pddl+.yacc"
    {log_error(E_WARNING,"Unrecognised requirements declaration ");
       (yyval.t_pddl_req_flag)= 0; delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 282:
#line 1325 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 283:
#line 1329 "../pddl+.yacc"
    {(yyval.t_type_list)=(yyvsp[(3) - (4)].t_type_list); requires(E_TYPING);;}
    break;

  case 284:
#line 1339 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(11) - (12)].t_problem); (yyval.t_problem)->name = (yyvsp[(5) - (12)].cp); (yyval.t_problem)->domain_name = (yyvsp[(9) - (12)].cp);
		if (types_used && !types_defined) {
			yyerrok; log_error(E_FATAL,"Syntax error in problem file - types used, but no :types section in domain file."); 
		}

	;}
    break;

  case 285:
#line 1346 "../pddl+.yacc"
    {yyerrok; (yyval.t_problem)=NULL;
       	log_error(E_FATAL,"Syntax error in problem definition."); ;}
    break;

  case 286:
#line 1352 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 287:
#line 1353 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->objects= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 288:
#line 1354 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->initial_state= (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 289:
#line 1355 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->the_goal= (yyvsp[(1) - (2)].t_goal);;}
    break;

  case 290:
#line 1357 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 291:
#line 1358 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->metric= (yyvsp[(1) - (2)].t_metric);;}
    break;

  case 292:
#line 1359 "../pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->length= (yyvsp[(1) - (2)].t_length_spec);;}
    break;

  case 293:
#line 1360 "../pddl+.yacc"
    {(yyval.t_problem)=new problem;;}
    break;

  case 294:
#line 1363 "../pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 295:
#line 1366 "../pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 296:
#line 1369 "../pddl+.yacc"
    {(yyval.vtab) = current_analysis->buildOpTab();;}
    break;

  case 297:
#line 1372 "../pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(3) - (4)].t_goal);delete (yyvsp[(2) - (4)].vtab);;}
    break;

  case 298:
#line 1377 "../pddl+.yacc"
    { (yyval.t_metric)= new metric_spec((yyvsp[(3) - (5)].t_optimization),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 299:
#line 1379 "../pddl+.yacc"
    {yyerrok; 
        log_error(E_FATAL,"Syntax error in metric declaration.");
        (yyval.t_metric)= NULL; ;}
    break;

  case 300:
#line 1386 "../pddl+.yacc"
    {(yyval.t_length_spec)= new length_spec(E_BOTH,(yyvsp[(4) - (7)].ival),(yyvsp[(6) - (7)].ival));;}
    break;

  case 301:
#line 1389 "../pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_SERIAL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 302:
#line 1393 "../pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_PARALLEL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 303:
#line 1399 "../pddl+.yacc"
    {(yyval.t_optimization)= E_MINIMIZE;;}
    break;

  case 304:
#line 1400 "../pddl+.yacc"
    {(yyval.t_optimization)= E_MAXIMIZE;;}
    break;

  case 305:
#line 1405 "../pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(2) - (3)].t_expression);;}
    break;

  case 306:
#line 1406 "../pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term);;}
    break;

  case 307:
#line 1407 "../pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_num_expression);;}
    break;

  case 308:
#line 1408 "../pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 309:
#line 1410 "../pddl+.yacc"
    {(yyval.t_expression) = new violation_term((yyvsp[(3) - (4)].cp));;}
    break;

  case 310:
#line 1411 "../pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 311:
#line 1415 "../pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 312:
#line 1416 "../pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 313:
#line 1417 "../pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 314:
#line 1418 "../pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 315:
#line 1422 "../pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 316:
#line 1424 "../pddl+.yacc"
    {(yyval.t_expression) = new plus_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 317:
#line 1428 "../pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 318:
#line 1430 "../pddl+.yacc"
    {(yyval.t_expression) = new mul_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 319:
#line 1436 "../pddl+.yacc"
    {(yyval.t_plan)= (yyvsp[(2) - (2)].t_plan); 
         (yyval.t_plan)->push_front((yyvsp[(1) - (2)].t_step)); ;}
    break;

  case 320:
#line 1439 "../pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].fval));;}
    break;

  case 321:
#line 1441 "../pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].ival));;}
    break;

  case 322:
#line 1443 "../pddl+.yacc"
    {(yyval.t_plan)= new plan;;}
    break;

  case 323:
#line 1448 "../pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(3) - (3)].t_step); 
         (yyval.t_step)->start_time_given=1; 
         (yyval.t_step)->start_time=(yyvsp[(1) - (3)].fval);;}
    break;

  case 324:
#line 1452 "../pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(1) - (1)].t_step);
	 (yyval.t_step)->start_time_given=0;;}
    break;

  case 325:
#line 1458 "../pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (4)].t_step); 
	 (yyval.t_step)->duration_given=1;
         (yyval.t_step)->duration= (yyvsp[(3) - (4)].fval);;}
    break;

  case 326:
#line 1462 "../pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (1)].t_step);
         (yyval.t_step)->duration_given=0;;}
    break;

  case 327:
#line 1468 "../pddl+.yacc"
    {(yyval.t_step)= new plan_step( 
              current_analysis->op_tab.symbol_get((yyvsp[(2) - (4)].cp)), 
	      (yyvsp[(3) - (4)].t_const_symbol_list)); delete [] (yyvsp[(2) - (4)].cp);
      ;}
    break;

  case 328:
#line 1475 "../pddl+.yacc"
    {(yyval.fval)= (yyvsp[(1) - (1)].fval);;}
    break;

  case 329:
#line 1476 "../pddl+.yacc"
    {(yyval.fval)= (float) (yyvsp[(1) - (1)].ival);;}
    break;


/* Line 1267 of yacc.c.  */
#line 4223 "pddl+.cpp"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;


  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse look-ahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse look-ahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#ifndef yyoverflow
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEOF && yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}


#line 1479 "../pddl+.yacc"


#include <cstdio>
#include <iostream>
int line_no= 1;
using std::istream;
#include "lex.yy.cc"

namespace VAL {
extern yyFlexLexer* yfl;
};


int yyerror(char * s)
{
    return 0;
}

int yylex()
{
    return yfl->yylex();
}

