#include "evaluator.hpp"

using namespace std;
using namespace parser;

namespace
{
  template <typename T>
  T Clamp(T minValue, T maxValue, T value)
  {
    return value < minValue ? minValue : value > maxValue ? maxValue : value;
  }

  template <typename T, typename U>
  T Lerp(T a, T b, U v)
  {
    return (T)((1 - v) * a + v * b);
  }

}


namespace eval
{
  static int BINOP_PRIO[4] = { 1, 1, 2, 2 };

  //------------------------------------------------------------------------------
  bool Parse(const char* str, vector<Token>* expression)
  {
    InputBuffer buf(str, strlen(str));

    while (!buf.Eof())
    {
      buf.SkipWhitespace();

      //char ch;
      float value;
      bool res;
      int idx;
      string fn;
      // woot, comma operator!
      if (value = parser::ParseFloat(buf, &res), res)
      {
        expression->push_back(Token(value));
      }
      else if (buf.IsOneOfIdx("+-*/", 4, &idx) && idx != -1)
      {
        expression->push_back(Token(Token::BinOp(idx)));
      }
      else if (buf.ConsumeIf('(') )
      {
        expression->push_back(Token(Token::Type::LeftParen));
      }
      else if (buf.ConsumeIf(')'))
      {
        expression->push_back(Token(Token::Type::RightParen));
      }
      else if (buf.ConsumeIf(','))
      {
        expression->push_back(Token(Token::Type::Comma));
      }
      else if (fn = parser::ParseIdentifier(buf, false, &res), res)
      {
        // differentiate between function calls and vars
        if (!buf.Eof() &&  buf.Peek() == '(')
          expression->push_back(Token(Token::Type::FuncCall, fn));
        else
          expression->push_back(Token(Token::Type::Var, fn));
      }
      else
      {
        LOG_WARN("Error tokenizing string", str);
        return false;
      }
    }
    return true;
  }

  //------------------------------------------------------------------------------
  Evaluator::Evaluator()
  {
    RegisterFunction("min", 2, [](eval::Evaluator* eval) {
      eval->PushValue(min(eval->PopValue(), eval->PopValue()));
    });

    RegisterFunction("max", 2, [](eval::Evaluator* eval) {
      eval->PushValue(max(eval->PopValue(), eval->PopValue()));
    });

    RegisterFunction("sin", 1, [](eval::Evaluator* eval) {
      eval->PushValue(sin(eval->PopValue()));
    });

    RegisterFunction("cos", 1, [](eval::Evaluator* eval) {
      eval->PushValue(cos(eval->PopValue()));
    });

    // arguments are stored in LIFO order
    RegisterFunction("step", 2, [](eval::Evaluator* e) {
      // step(cutoff, t)
      float t = e->PopValue();
      float cutoff = e->PopValue();
      e->PushValue(t >= cutoff ? 1.f : 0.f);
    });

    RegisterFunction("pulse", 3, [](eval::Evaluator* e) {
      // pulse(start, stop, t)
      float t = e->PopValue();
      float stop = e->PopValue();
      float start = e->PopValue();
      e->PushValue((t >= start && t < stop) ? 1.f : 0.f);
    });

    RegisterFunction("edecay", 2, [](eval::Evaluator* e) {
      // edecay(k, t)
      float t = e->PopValue();
      float k = e->PopValue();
      e->PushValue(exp(-k*t));
    });

    RegisterFunction("ldecay", 2, [](eval::Evaluator* e) {
      // Lerp between 1..0
      float t = e->PopValue();
      float k = e->PopValue();
      float s = Clamp(0.f, 1.f, t * k);
      e->PushValue(Lerp(1.f, 0.f, s));
    });

    RegisterFunction("lfade_in", 3, [](eval::Evaluator* e) {
      // lfade_in(a, b, t)
      // Linear fade in between a, b
      float t = e->PopValue();
      float b = e->PopValue();
      float a = e->PopValue();
      float res;
      if (t <= a)
        res = 0;
      else if (t >= b)
        res = 1;
      else
        res = Lerp(0.f, 1.f, (t - a) / (b - a));
      e->PushValue(res);
    });


  }

  //------------------------------------------------------------------------------
  void Evaluator::RegisterFunction(const string& name, int numArgs, const fnFunction& fn)
  {
    functions[name] = UserFunction{ numArgs, fn };
  }

  //------------------------------------------------------------------------------
  void Evaluator::SetConstant(const string& name, float value)
  {
    constants[name] = value;
  }

  //------------------------------------------------------------------------------
  float Evaluator::PopValue()
  {
    if (resultStack.empty())
    {
      errorString = "Too few operands";
      success = false;
      return 0;
    }

    float v = resultStack.back().constant;
    resultStack.pop_back();
    return v;
  }

  //------------------------------------------------------------------------------
  void Evaluator::PushValue(float value)
  {
    resultStack.push_back(Token{ value });
  }

  //------------------------------------------------------------------------------
  Token Evaluator::PopOperator()
  {
    if (operatorStack.empty())
    {
      errorString = "Too few operators";
      success = false;
      return 0;
    }

    Token t = operatorStack.back();
    operatorStack.pop_back();
    return t;
  }

  //------------------------------------------------------------------------------
  void Evaluator::ApplyBinOp(Token::BinOp op)
  {
    if (resultStack.size() < 2)
    {
      errorString = "Missing operands";
      success = false;
      return;
    }

    float b = PopValue();
    float a = PopValue();
    switch (op)
    {
      case Token::BinOpAdd: PushValue(a + b); break;
      case Token::BinOpSub: PushValue(a - b); break;
      case Token::BinOpMul: PushValue(a * b); break;
      case Token::BinOpDiv: PushValue(a / b); break;
      default:
      {
        errorString = "Unknown bin-op";
        success = false;
        break;
      }
    }
  }

  //------------------------------------------------------------------------------
  void Evaluator::LookupVar(const Token& t, const Environment* env)
  {
    const string& name = t.name;
    if (env)
    {
      auto it = env->constants.find(name);
      if (it != env->constants.end())
      {
        PushValue(it->second);
        return;
      }
    }

    if (constants.count(name) == 0)
    {
      errorString = "Unknown constant: " + name;
      return;
    }

    PushValue(constants[name]);
  }

  //------------------------------------------------------------------------------
  void Evaluator::InvokeFunction(const Token& t, const Environment* env)
  {
    const string& name = t.name;

    const UserFunction* fn = nullptr;

    // First check the environment for the function, then check built-ins
    if (env)
    {
      auto it = env->functions.find(name);
      if (it != env->functions.end())
        fn = &it->second;
    }

    if (!fn)
    {
      auto it = functions.find(name);
      if (it != functions.end())
        fn = &it->second;
    }

    if (!fn)
    {
      errorString = "Unknown function: " + name;
      return;
    }

    // Verify that we have enough arguments on the stack
    if (resultStack.size() < fn->numArgs)
    {
      errorString = "Too few arguments for user function: " + name;
      success = false;
      return;
    }

    fn->fn(this);
  }

  //------------------------------------------------------------------------------
  void Evaluator::ApplyUntilLeftParen(bool discardParen)
  {
    while (!operatorStack.empty())
    {
      Token token = PopOperator();
      if (token.type == Token::Type::LeftParen)
      {
        if (!discardParen)
          operatorStack.push_back(token);
        break;
      }
      else
      {
        ApplyBinOp(token.binOp);
      }
    }
  }

  //------------------------------------------------------------------------------
  float Evaluator::EvaluateFromString(const char* str, const Environment* env)
  {
    vector<Token> expr;
    Parse(str, &expr);
    return Evaluate(expr, env);
  }

  //------------------------------------------------------------------------------
  float Evaluator::Evaluate(const vector<Token>& expression, const Environment* env)
  {

    // Now perform the actual shunting :)
    for (size_t i = 0; i < expression.size(); ++i)
    {
      if (!success)
      {
        return 0;
      }

      const Token& t = expression[i];

      if (t.type == Token::Type::BinOp)
      {
        // Apply any higher priority operators
        int prio = BINOP_PRIO[t.binOp];
        while (!operatorStack.empty())
        {
          const Token& op = operatorStack.back();
          if (op.type == Token::Type::BinOp && BINOP_PRIO[op.binOp] >= prio)
          {
            ApplyBinOp(op.binOp);
            operatorStack.pop_back();
          }
          else
            break;
        }

        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::Constant)
      {
        resultStack.push_back(t);
      }
      else if (t.type == Token::Type::FuncCall)
      {
        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::LeftParen)
      {
        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::Comma)
      {
        // apply all the operators until the left paren
        ApplyUntilLeftParen(false);
      }
      else if (t.type == Token::Type::RightParen)
      {
        ApplyUntilLeftParen(true);
        if (!operatorStack.empty())
        {
          // if the token at the top of the operator stack is a function call,
          // then invoke it
          Token t = operatorStack.back();
          if (t.type == Token::Type::FuncCall)
          {
            InvokeFunction(t, env);
            operatorStack.pop_back();
          }
        }
      }
      else if (t.type == Token::Type::Var)
      {
        LookupVar(t, env);
      }
    }

    // apply all the remaining operators
    while (!operatorStack.empty())
    {
      ApplyBinOp(PopOperator().binOp);
    }

    return PopValue();
  }
}
