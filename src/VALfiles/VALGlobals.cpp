namespace VAL {
class analysis;
class parse_category;
}

class yyFlexLexer;

namespace VAL {

parse_category* top_thing = nullptr;
analysis* current_analysis = nullptr;
yyFlexLexer* yfl = nullptr;

}  // namespace VAL

char* current_filename = nullptr;
