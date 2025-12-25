// ==============================================
// LLM SUMMARIZATION SYSTEM IN GLEAM
// Summarizing 5 popular LLMs with functional purity
// ==============================================

import gleam/io
import gleam/list
import gleam/string
import gleam/dict
import gleam/option.{Option, Some, None}
import gleam/result.{Result, Ok, Error}
import gleam/bit_array
import gleam/dynamic

// ============= DATA STRUCTURES =============

type LlmProvider {
  Gemini
  ChatGPT
  DeepSeek
  Claude
  Grok
}

type LanguageSupport {
  English
  Chinese
  Japanese
  Korean
  Spanish
  French
  German
  Multi
}

type PricingModel {
  FreeTier(credits: Int)
  PayPerToken(cost_per_million: Float)
  Subscription(monthly_fee: Float)
  Enterprise(contact_sales: Bool)
}

type Capability {
  CodeGeneration
  Mathematics
  CreativeWriting
  Translation
  Reasoning
  Multimodal
  RealTime
  FileUpload
  InternetSearch
  FunctionCalling
}

type LlmModel {
  LlmModel(
    provider: LlmProvider,
    name: String,
    release_year: Int,
    max_context: Int, // in tokens
    supported_languages: List(LanguageSupport),
    pricing: PricingModel,
    key_features: List(Capability),
    strengths: List(String),
    limitations: List(String),
    best_for: List(String),
    ethical_approach: String,
    unique_characteristics: List(String)
  )
}

// ============= LLM DATABASE =============

let gemini_model = LlmModel(
  provider: Gemini,
  name: "Gemini Ultra",
  release_year: 2023,
  max_context: 1_000_000,
  supported_languages: [English, Chinese, Japanese, Korean, Spanish, French, German, Multi],
  pricing: PayPerToken(cost_per_million: 0.00025),
  key_features: [Multimodal, CodeGeneration, Mathematics, Reasoning, InternetSearch],
  strengths: [
    "Native multimodal processing",
    "Strong mathematical reasoning",
    "Seamless code generation across languages",
    "Real-time web search integration",
    "Excellent translation quality"
  ],
  limitations: [
    "Limited transparency in training data",
    "Occasional hallucination in creative tasks",
    "Sometimes overly cautious in responses"
  ],
  best_for: [
    "Multimodal applications",
    "Mathematical problem solving",
    "Real-time information retrieval",
    "Code generation and review",
    "Educational applications"
  ],
  ethical_approach: "Proactive harm reduction with extensive safety layers",
  unique_characteristics: [
    "Built from ground-up as multimodal",
    "Direct integration with Google Search",
    "Native support for 100+ languages"
  ]
)

let chatgpt_model = LlmModel(
  provider: ChatGPT,
  name: "GPT-4 Turbo",
  release_year: 2022,
  max_context: 128_000,
  supported_languages: [English, Chinese, Japanese, Spanish, French, German, Multi],
  pricing: PayPerToken(cost_per_million: 0.01),
  key_features: [CodeGeneration, CreativeWriting, Reasoning, FunctionCalling, FileUpload],
  strengths: [
    "Extensive fine-tuning for conversational quality",
    "Excellent code generation and debugging",
    "Strong plugin/API ecosystem",
    "Consistent output formatting",
    "Large developer community"
  ],
  limitations: [
    "Knowledge cutoff (April 2023 for GPT-4)",
    "Can be verbose",
    "Occasional factual inaccuracies",
    "Limited multimodal capabilities in base model"
  ],
  best_for: [
    "Chatbot development",
    "Content creation and editing",
    "Programming assistance",
    "API integration",
    "Creative writing"
  ],
  ethical_approach: "Constitutional AI with human feedback reinforcement",
  unique_characteristics: [
    "Massive-scale RLHF training",
    "Extensive plugin ecosystem",
    "First-mover advantage in consumer AI"
  ]
)

let deepseek_model = LlmModel(
  provider: DeepSeek,
  name: "DeepSeek V2",
  release_year: 2024,
  max_context: 128_000,
  supported_languages: [Chinese, English, Multi],
  pricing: FreeTier(credits: 1000),
  key_features: [CodeGeneration, Mathematics, Reasoning, FileUpload],
  strengths: [
    "Exceptional Chinese language understanding",
    "Strong mathematical reasoning capabilities",
    "Cost-effective pricing",
    "Good code generation for Asian languages",
    "Efficient token usage"
  ],
  limitations: [
    "Primary focus on Chinese/English",
    "Limited multimodal capabilities",
    "Smaller community compared to Western LLMs",
    "Less extensive documentation"
  ],
  best_for: [
    "Chinese language applications",
    "Mathematical problem solving",
    "Cost-sensitive projects",
    "Academic research in Asia",
    "Code generation for Chinese projects"
  ],
  ethical_approach: "Cultural sensitivity with Chinese values integration",
  unique_characteristics: [
    "Optimized for Chinese language and culture",
    "Cost leadership strategy",
    "Strong focus on academic applications"
  ]
)

let claude_model = LlmModel(
  provider: Claude,
  name: "Claude 3 Opus",
  release_year: 2024,
  max_context: 200_000,
  supported_languages: [English, Multi],
  pricing: PayPerToken(cost_per_million: 0.075),
  key_features: [CreativeWriting, Reasoning, FileUpload, Mathematics],
  strengths: [
    "Exceptional reasoning and analysis",
    "Strong ethical and safety alignment",
    "Excellent creative writing quality",
    "Good at following complex instructions",
    "Low hallucination rate"
  ],
  limitations: [
    "Most expensive model",
    "Conservative in creative tasks",
    "Limited code generation compared to specialists",
    "Primarily English-focused"
  ],
  best_for: [
    "Legal and compliance applications",
    "Academic writing and analysis",
    "Creative content generation",
    "Complex reasoning tasks",
    "Safety-critical applications"
  ],
  ethical_approach: "Constitutional AI with harm minimization as core principle",
  unique_characteristics: [
    "Strong emphasis on AI safety and ethics",
    "Excellent at following detailed instructions",
    "Designed for enterprise trust and reliability"
  ]
)

let grok_model = LlmModel(
  provider: Grok,
  name: "Grok-1.5",
  release_year: 2023,
  max_context: 131_072,
  supported_languages: [English, Multi],
  pricing: Subscription(monthly_fee: 16.0),
  key_features: [RealTime, InternetSearch, CreativeWriting, Reasoning],
  strengths: [
    "Real-time information access",
    "Humorous and engaging personality",
    "Direct integration with X platform",
    "Uncensored creative responses",
    "Current events knowledge"
  ],
  limitations: [
    "Limited language support",
    "Less refined for professional tasks",
    "Can be too informal for some applications",
    "Limited enterprise features"
  ],
  best_for: [
    "Social media content creation",
    "Real-time information synthesis",
    "Entertainment and humor generation",
    "Current events analysis",
    "Casual conversation"
  ],
  ethical_approach: "Maximally helpful with minimal content filtering",
  unique_characteristics: [
    "Real-time knowledge integration",
    "Witty and humorous personality",
    "Direct access to X platform data",
    "Designed for engaging conversation"
  ]
)

// ============= UTILITY FUNCTIONS =============

fn pricing_to_string(pricing: PricingModel) -> String {
  case pricing {
    FreeTier(credits) -> "Free tier with " <> int.to_string(credits) <> " credits/month"
    PayPerToken(cost) -> 
      "$" <> float.to_string(cost) <> "/million tokens"
    Subscription(fee) -> 
      "$" <> float.to_string(fee) <> "/month subscription"
    Enterprise(contact) -> 
      "Enterprise pricing (contact sales)"
  }
}

fn capabilities_to_string(capabilities: List(Capability)) -> String {
  let format_cap = fn(cap: Capability) -> String {
    case cap {
      CodeGeneration -> "Code Generation"
      Mathematics -> "Mathematics"
      CreativeWriting -> "Creative Writing"
      Translation -> "Translation"
      Reasoning -> "Reasoning"
      Multimodal -> "Multimodal"
      RealTime -> "Real-time"
      FileUpload -> "File Upload"
      InternetSearch -> "Internet Search"
      FunctionCalling -> "Function Calling"
    }
  }
  
  capabilities
  |> list.map(format_cap)
  |> list.join(" • ")
}

fn languages_to_string(languages: List(LanguageSupport)) -> String {
  let format_lang = fn(lang: LanguageSupport) -> String {
    case lang {
      English -> "English"
      Chinese -> "Chinese"
      Japanese -> "Japanese"
      Korean -> "Korean"
      Spanish -> "Spanish"
      French -> "French"
      German -> "German"
      Multi -> "100+ Languages"
    }
  }
  
  languages
  |> list.map(format_lang)
  |> list.join(", ")
}

fn provider_to_string(provider: LlmProvider) -> String {
  case provider {
    Gemini -> "Google Gemini"
    ChatGPT -> "OpenAI ChatGPT"
    DeepSeek -> "DeepSeek"
    Claude -> "Anthropic Claude"
    Grok -> "xAI Grok"
  }
}

// ============= SUMMARIZATION FUNCTIONS =============

fn generate_summary(model: LlmModel) -> String {
  let LlmModel(
    provider,
    name,
    release_year,
    max_context,
    supported_languages,
    pricing,
    key_features,
    strengths,
    limitations,
    best_for,
    ethical_approach,
    unique_characteristics
  ) = model
  
  string.concat([
    "╔═══════════════════════════════════════════════════════════╗\n",
    "║ ", provider_to_string(provider), " - ", name, " (", int.to_string(release_year), ") ",
    "║\n",
    "╚═══════════════════════════════════════════════════════════╝\n\n",
    "📊 CONTEXT & LANGUAGES:\n",
    "  • Context Window: ", int.to_string(max_context), " tokens\n",
    "  • Languages: ", languages_to_string(supported_languages), "\n",
    "  • Pricing: ", pricing_to_string(pricing), "\n\n",
    "🚀 KEY CAPABILITIES:\n",
    "  ", capabilities_to_string(key_features), "\n\n",
    "⭐ STRENGTHS:\n",
    list.map(strengths, fn(s) { "  • " <> s <> "\n" })
    |> string.concat(),
    "\n",
    "⚠️ LIMITATIONS:\n",
    list.map(limitations, fn(s) { "  • " <> s <> "\n" })
    |> string.concat(),
    "\n",
    "🎯 BEST SUITED FOR:\n",
    list.map(best_for, fn(s) { "  • " <> s <> "\n" })
    |> string.concat(),
    "\n",
    "⚖️ ETHICAL APPROACH:\n",
    "  ", ethical_approach, "\n\n",
    "🔍 UNIQUE CHARACTERISTICS:\n",
    list.map(unique_characteristics, fn(s) { "  • " <> s <> "\n" })
    |> string.concat()
  ])
}

fn generate_comparison_table(models: List(LlmModel)) -> String {
  let header = [
    "Provider",
    "Context",
    "Languages",
    "Pricing",
    "Key Feature",
    "Best Use Case"
  ]
  
  let format_row = fn(model: LlmModel) -> List(String) {
    let LlmModel(provider, name, _, max_context, languages, pricing, features, _, _, best_for, _, _) = model
    
    let primary_feature = list.first(features)
      |> option.unwrap(FunctionCalling)
    
    let primary_use_case = list.first(best_for)
      |> option.unwrap("General Purpose")
    
    [
      provider_to_string(provider),
      int.to_string(max_context) <> "K",
      int.to_string(list.length(languages)) <> " primary",
      case pricing {
        FreeTier(_) -> "Free"
        PayPerToken(cost) when cost < 0.001 -> "Very Low"
        PayPerToken(cost) when cost < 0.01 -> "Low"
        PayPerToken(_) -> "Medium"
        Subscription(fee) when fee < 10.0 -> "Low Sub"
        Subscription(_) -> "Premium"
        Enterprise(_) -> "Enterprise"
      },
      case primary_feature {
        Multimodal -> "Multimodal"
        RealTime -> "Real-time"
        CodeGeneration -> "Code Gen"
        Mathematics -> "Math"
        CreativeWriting -> "Creative"
        Reasoning -> "Reasoning"
        _ -> "General"
      },
      primary_use_case
    ]
  }
  
  let rows = list.map(models, format_row)
  
  let max_lengths = list.fold(header, [0, 0, 0, 0, 0, 0], fn(acc, _) {
    // Calculate max column widths
    let col0 = list.max([string.length(header[0]), list.length(list.map(rows, fn(r) { string.length(r[0]) }))])
    let col1 = list.max([string.length(header[1]), list.length(list.map(rows, fn(r) { string.length(r[1]) }))])
    let col2 = list.max([string.length(header[2]), list.length(list.map(rows, fn(r) { string.length(r[2]) }))])
    let col3 = list.max([string.length(header[3]), list.length(list.map(rows, fn(r) { string.length(r[3]) }))])
    let col4 = list.max([string.length(header[4]), list.length(list.map(rows, fn(r) { string.length(r[4]) }))])
    let col5 = list.max([string.length(header[5]), list.length(list.map(rows, fn(r) { string.length(r[5]) }))])
    
    [col0, col1, col2, col3, col4, col5]
  })
  
  let [col0, col1, col2, col3, col4, col5] = max_lengths
  
  let pad = fn(text: String, length: Int) -> String {
    let padding = length - string.length(text)
    text <> string.repeat(" ", padding)
  }
  
  let separator = "+" <> string.repeat("-", col0 + 2) <> "+" <> 
                  string.repeat("-", col1 + 2) <> "+" <>
                  string.repeat("-", col2 + 2) <> "+" <>
                  string.repeat("-", col3 + 2) <> "+" <>
                  string.repeat("-", col4 + 2) <> "+" <>
                  string.repeat("-", col5 + 2) <> "+\n"
  
  let header_row = "| " <> pad(header[0], col0) <> " | " <>
                   pad(header[1], col1) <> " | " <>
                   pad(header[2], col2) <> " | " <>
                   pad(header[3], col3) <> " | " <>
                   pad(header[4], col4) <> " | " <>
                   pad(header[5], col5) <> " |\n"
  
  let data_rows = list.map(rows, fn(row) {
    "| " <> pad(row[0], col0) <> " | " <>
    pad(row[1], col1) <> " | " <>
    pad(row[2], col2) <> " | " <>
    pad(row[3], col3) <> " | " <>
    pad(row[4], col4) <> " | " <>
    pad(row[5], col5) <> " |\n"
  })
  |> string.concat()
  
  separator <> header_row <> separator <> data_rows <> separator
}

fn calculate_score(model: LlmModel, weights: Dict(String, Float)) -> Float {
  let LlmModel(_, _, _, max_context, languages, pricing, features, strengths, limitations, _, _, _) = model
  
  // Context score (normalized to 200K max)
  let context_score = float.div(int.to_float(max_context), 200000.0) * 100.0
  
  // Language diversity score
  let language_score = int.to_float(list.length(languages)) * 15.0
  
  // Pricing score (lower is better)
  let pricing_score = case pricing {
    FreeTier(_) -> 100.0
    PayPerToken(cost) -> 100.0 - (cost * 10000.0)
    Subscription(fee) -> 100.0 - (fee * 2.0)
    Enterprise(_) -> 50.0
  }
  
  // Features score
  let feature_score = int.to_float(list.length(features)) * 10.0
  
  // Strengths vs limitations ratio
  let balance_score = 
    int.to_float(list.length(strengths)) / 
    (int.to_float(list.length(limitations)) + 1.0) * 30.0
  
  // Apply weights if provided
  let context_weight = dict.get(weights, "context") |> option.unwrap(0.2)
  let language_weight = dict.get(weights, "language") |> option.unwrap(0.15)
  let pricing_weight = dict.get(weights, "pricing") |> option.unwrap(0.25)
  let feature_weight = dict.get(weights, "feature") |> option.unwrap(0.25)
  let balance_weight = dict.get(weights, "balance") |> option.unwrap(0.15)
  
  context_score * context_weight +
  language_score * language_weight +
  pricing_score * pricing_weight +
  feature_score * feature_weight +
  balance_score * balance_weight
}

fn find_best_for_use_case(use_case: String, models: List(LlmModel)) -> Result(LlmModel, String) {
  let use_case_lower = string.lowercase(use_case)
  
  let scores = list.map(models, fn(model) {
    let LlmModel(_, _, _, _, _, _, _, _, _, best_for, _, _) = model
    
    // Check if use case is in best_for list
    let matches = list.any(best_for, fn(bf) {
      string.contains(string.lowercase(bf), use_case_lower)
    })
    
    let score = if matches { 100.0 } else { 0.0 }
    
    #(model, score)
  })
  
  let sorted = list.sort(scores, fn(a, b) {
    let #(_, score_a) = a
    let #(_, score_b) = b
    score_a > score_b
  })
  
  case list.first(sorted) {
    Some(#(model, score)) when score > 0.0 -> Ok(model)
    _ -> Error("No suitable model found for: " <> use_case)
  }
}

fn create_recommendation_engine(models: List(LlmModel)) -> 
  fn(requirements: Dict(String, Bool)) -> List(LlmModel) {
  
  let weights = dict.from_list([
    #("code_generation", 0.3),
    #("multimodal", 0.25),
    #("real_time", 0.2),
    #("low_cost", 0.15),
    #("safety", 0.1)
  ])
  
  fn(requirements: Dict(String, Bool)) -> List(LlmModel) {
    let score_model = fn(model: LlmModel) -> #(LlmModel, Float) {
      let LlmModel(_, _, _, _, _, _, features, _, _, _, ethical_approach, _) = model
      
      let mutable score = 0.0
      
      // Check code generation requirement
      case dict.get(requirements, "code_generation") {
        Some(true) -> 
          if list.any(features, fn(f) { f == CodeGeneration }) {
            score = score +. dict.get(weights, "code_generation") |> option.unwrap(0.0)
          }
        _ -> score
      }
      
      // Check multimodal requirement
      case dict.get(requirements, "multimodal") {
        Some(true) -> 
          if list.any(features, fn(f) { f == Multimodal }) {
            score = score +. dict.get(weights, "multimodal") |> option.unwrap(0.0)
          }
        _ -> score
      }
      
      // Check real-time requirement
      case dict.get(requirements, "real_time") {
        Some(true) -> 
          if list.any(features, fn(f) { f == RealTime }) {
            score = score +. dict.get(weights, "real_time") |> option.unwrap(0.0)
          }
        _ -> score
      }
      
      // Check low cost requirement
      case dict.get(requirements, "low_cost") {
        Some(true) -> 
          score = score +. (calculate_score(model, dict.from_list([
            #("pricing", 0.5),
            #("feature", 0.3),
            #("balance", 0.2)
          ])) / 100.0) *. dict.get(weights, "low_cost") |> option.unwrap(0.0)
        _ -> score
      }
      
      // Check safety requirement
      case dict.get(requirements, "safety") {
        Some(true) -> 
          if string.contains(string.lowercase(ethical_approach), "safety") ||
             string.contains(string.lowercase(ethical_approach), "ethical") ||
             string.contains(string.lowercase(ethical_approach), "harm") {
            score = score +. dict.get(weights, "safety") |> option.unwrap(0.0)
          }
        _ -> score
      }
      
      #(model, score)
    }
    
    let scored_models = list.map(models, score_model)
    
    let sorted = list.sort(scored_models, fn(a, b) {
      let #(_, score_a) = a
      let #(_, score_b) = b
      score_a > score_b
    })
    
    // Return models with score > 0
    list.filter_map(sorted, fn(pair) {
      let #(model, score) = pair
      if score > 0.0 { Some(model) } else { None }
    })
  }
}

// ============= ANALYSIS FUNCTIONS =============

fn analyze_language_distribution(models: List(LlmModel)) -> Dict(String, Int) {
  let extract_languages = fn(model: LlmModel) -> List(LanguageSupport) {
    let LlmModel(_, _, _, _, languages, _, _, _, _, _, _, _) = model
    languages
  }
  
  let all_languages = list.map(models, extract_languages)
    |> list.flatten
  
  let count_language = fn(lang: LanguageSupport, counts: Dict(String, Int)) -> Dict(String, Int) {
    let lang_name = case lang {
      English -> "English"
      Chinese -> "Chinese"
      Japanese -> "Japanese"
      Korean -> "Korean"
      Spanish -> "Spanish"
      French -> "French"
      German -> "German"
      Multi -> "Multi"
    }
    
    let current = dict.get(counts, lang_name) |> option.unwrap(0)
    dict.insert(counts, lang_name, current + 1)
  }
  
  list.fold(all_languages, dict.new(), count_language)
}

fn find_most_cost_effective(models: List(LlmModel)) -> Option(LlmModel) {
  let calculate_value = fn(model: LlmModel) -> #(LlmModel, Float) {
    let score = calculate_score(model, dict.from_list([
      #("pricing", 0.4),
      #("feature", 0.3),
      #("context", 0.2),
      #("language", 0.1)
    ]))
    #(model, score)
  }
  
  let scored = list.map(models, calculate_value)
  
  list.max_by(scored, fn(a, b) {
    let #(_, score_a) = a
    let #(_, score_b) = b
    score_a > score_b
  })
  |> option.map(fn(pair) {
    let #(model, _) = pair
    model
  })
}

fn generate_statistics_report(models: List(LlmModel)) -> String {
  let total_models = list.length(models)
  
  let avg_context = list.map(models, fn(m) {
    let LlmModel(_, _, _, max_context, _, _, _, _, _, _, _, _) = m
    max_context
  })
  |> list.fold(0.0, fn(acc, ctx) { acc +. int.to_float(ctx) })
  |> fn(sum) { sum /. int.to_float(total_models) }
  
  let free_models = list.filter(models, fn(m) {
    let LlmModel(_, _, _, _, _, pricing, _, _, _, _, _, _) = m
    case pricing {
      FreeTier(_) -> True
      _ -> False
    }
  })
  |> list.length
  
  let multimodal_models = list.filter(models, fn(m) {
    let LlmModel(_, _, _, _, _, _, features, _, _, _, _, _) = m
    list.any(features, fn(f) { f == Multimodal })
  })
  |> list.length
  
  string.concat([
    "📈 LLM MARKET STATISTICS\n",
    "═══════════════════════\n",
    "Total Models Analyzed: ", int.to_string(total_models), "\n",
    "Average Context Window: ", float.to_string(avg_context), " tokens\n",
    "Models with Free Tier: ", int.to_string(free_models), "\n",
    "Multimodal Models: ", int.to_string(multimodal_models), "\n",
    "\n",
    "Language Support Distribution:\n"
  ])
}

// ============= MAIN PROGRAM =============

pub fn main() {
  let all_models = [gemini_model, chatgpt_model, deepseek_model, claude_model, grok_model]
  
  io.println("\n" <> string.repeat("=", 70))
  io.println("          COMPREHENSIVE LLM ANALYSIS REPORT")
  io.println("                5 Popular Large Language Models")
  io.println(string.repeat("=", 70) <> "\n")
  
  // Generate individual summaries
  io.println("🔍 INDIVIDUAL MODEL SUMMARIES")
  io.println(string.repeat("-", 70))
  
  list.each(all_models, fn(model) {
    let LlmModel(provider, name, _, _, _, _, _, _, _, _, _, _) = model
    io.println("\n" <> generate_summary(model))
  })
  
  // Generate comparison table
  io.println("\n📊 COMPARISON TABLE")
  io.println(string.repeat("-", 70))
  io.println(generate_comparison_table(all_models))
  
  // Generate statistics
  io.println("\n📈 MARKET ANALYSIS")
  io.println(string.repeat("-", 70))
  io.println(generate_statistics_report(all_models))
  
  // Language distribution
  let lang_dist = analyze_language_distribution(all_models)
  dict.fold(lang_dist, io.println(""), fn(key, value, _) {
    io.println("  • " <> key <> ": " <> int.to_string(value) <> " models")
  })
  
  // Recommendation examples
  io.println("\n🎯 RECOMMENDATION ENGINE EXAMPLES")
  io.println(string.repeat("-", 70))
  
  let recommender = create_recommendation_engine(all_models)
  
  io.println("\n1. For a low-cost coding assistant:")
  let coding_req = dict.from_list([
    #("code_generation", True),
    #("low_cost", True)
  ])
  let coding_recs = recommender(coding_req)
  list.each(coding_recs, fn(model) {
    let LlmModel(provider, name, _, _, _, _, _, _, _, _, _, _) = model
    io.println("   • " <> provider_to_string(provider) <> " - " <> name)
  })
  
  io.println("\n2. For a real-time news analyzer:")
  let news_req = dict.from_list([
    #("real_time", True),
    #("safety", True)
  ])
  let news_recs = recommender(news_req)
  list.each(news_recs, fn(model) {
    let LlmModel(provider, name, _, _, _, _, _, _, _, _, _, _) = model
    io.println("   • " <> provider_to_string(provider) <> " - " <> name)
  })
  
  io.println("\n3. For a multimodal creative tool:")
  let creative_req = dict.from_list([
    #("multimodal", True),
    #("code_generation", True)
  ])
  let creative_recs = recommender(creative_req)
  list.each(creative_recs, fn(model) {
    let LlmModel(provider, name, _, _, _, _, _, _, _, _, _, _) = model
    io.println("   • " <> provider_to_string(provider) <> " - " <> name)
  })
  
  // Find best value model
  io.println("\n💰 BEST VALUE RECOMMENDATION")
  io.println(string.repeat("-", 70))
  case find_most_cost_effective(all_models) {
    Some(model) ->
      let LlmModel(provider, name, _, _, _, pricing, _, _, _, _, _, _) = model
      io.println("Based on cost vs capabilities analysis:")
      io.println("  🏆 " <> provider_to_string(provider) <> " - " <> name)
      io.println("  💵 " <> pricing_to_string(pricing))
    None ->
      io.println("No clear winner for value")
  }
  
  // Use case matching
  io.println("\n🔎 USE CASE MATCHING")
  io.println(string.repeat("-", 70))
  
  let use_cases = [
    "Chinese language processing",
    "Legal document analysis",
    "Social media content",
    "Mathematical research",
    "API development"
  ]
  
  list.each(use_cases, fn(use_case) {
    io.println("\nFor: " <> use_case)
    case find_best_for_use_case(use_case, all_models) {
      Ok(model) ->
        let LlmModel(provider, name, _, _, _, _, _, _, _, best_for, _, _) = model
        io.println("  Recommended: " <> provider_to_string(provider) <> " - " <> name)
        io.println("  Best for: " <> list.first(best_for) |> option.unwrap("Various tasks"))
      Error(err) ->
        io.println("  " <> err)
    }
  })
  
  // Final scores
  io.println("\n⭐ FINAL SCORING (0-100)")
  io.println(string.repeat("-", 70))
  
  let weights = dict.from_list([
    #("context", 0.2),
    #("language", 0.15),
    #("pricing", 0.25),
    #("feature", 0.25),
    #("balance", 0.15)
  ])
  
  list.each(all_models, fn(model) {
    let LlmModel(provider, name, _, _, _, pricing, _, _, _, _, _, _) = model
    let score = calculate_score(model, weights)
    io.println(
      "  " <> provider_to_string(provider) <> 
      ": " <> float.to_string(score) <> 
      " (" <> pricing_to_string(pricing) <> ")"
    )
  })
  
  io.println("\n" <> string.repeat("=", 70))
  io.println("                    ANALYSIS COMPLETE")
  io.println("          " <> int.to_string(list.length(all_models)) <> " models evaluated")
  io.println(string.repeat("=", 70))
}

// ============= TEST FUNCTIONS =============

fn test_summarization() {
  let all_models = [gemini_model, chatgpt_model, deepseek_model, claude_model, grok_model]
  
  // Test that all models can be summarized
  list.each(all_models, fn(model) {
    let summary = generate_summary(model)
    assert string.length(summary) > 100
  })
  
  // Test comparison table generation
  let table = generate_comparison_table(all_models)
  assert string.contains(table, "Provider")
  assert string.contains(table, "Context")
  
  // Test scoring function
  let scores = list.map(all_models, fn(m) { calculate_score(m, dict.new()) })
  assert list.length(scores) == 5
  
  io.println("✅ All tests passed!")
}

// ============= EXPORT FUNCTIONS =============

pub fn get_model(provider: LlmProvider) -> Result(LlmModel, String) {
  case provider {
    Gemini -> Ok(gemini_model)
    ChatGPT -> Ok(chatgpt_model)
    DeepSeek -> Ok(deepseek_model)
    Claude -> Ok(claude_model)
    Grok -> Ok(grok_model)
  }
}

pub fn compare_models(provider1: LlmProvider, provider2: LlmProvider) -> String {
  let get_field = fn(model: LlmModel, field: String) -> String {
    let LlmModel(prov, name, year, ctx, langs, pricing, features, strengths, limits, best, ethical, unique) = model
    
    case field {
      "name" -> name
      "context" -> int.to_string(ctx)
      "languages" -> int.to_string(list.length(langs))
      "pricing" -> pricing_to_string(pricing)
      "features" -> int.to_string(list.length(features))
      "strengths" -> int.to_string(list.length(strengths))
      "ethical" -> ethical
      _ -> "N/A"
    }
  }
  
  case #(get_model(provider1), get_model(provider2)) {
    #(Ok(model1), Ok(model2)) ->
      string.concat([
        "Comparison: ", provider_to_string(provider1), " vs ", provider_to_string(provider2), "\n",
        string.repeat("-", 50), "\n",
        "Model Name: ", get_field(model1, "name"), " | ", get_field(model2, "name"), "\n",
        "Context: ", get_field(model1, "context"), " | ", get_field(model2, "context"), "\n",
        "Languages: ", get_field(model1, "languages"), " | ", get_field(model2, "languages"), "\n",
        "Features: ", get_field(model1, "features"), " | ", get_field(model2, "features"), "\n",
        "Strengths: ", get_field(model1, "strengths"), " | ", get_field(model2, "strengths"), "\n"
      ])
    #(Error(e), _) -> "Error: " <> e
    #(_, Error(e)) -> "Error: " <> e
  }
}

pub fn recommend_model(
  needs_code: Bool,
  needs_multimodal: Bool,
  needs_realtime: Bool,
  budget: String
) -> List(String) {
  let all_models = [gemini_model, chatgpt_model, deepseek_model, claude_model, grok_model]
  
  let filter_by_budget = fn(model: LlmModel) -> Bool {
    let LlmModel(_, _, _, _, _, pricing, _, _, _, _, _, _) = model
    
    case (pricing, budget) {
      #(FreeTier(_), "free") -> True
      #(PayPerToken(cost), "low") when cost < 0.001 -> True
      #(PayPerToken(cost), "medium") when cost < 0.01 -> True
      #(Subscription(fee), "low") when fee < 10.0 -> True
      #(Subscription(fee), "medium") when fee < 50.0 -> True
      #(Enterprise(_), "enterprise") -> True
      #(_, "any") -> True
      _ -> False
    }
  }
  
  let filter_by_features = fn(model: LlmModel) -> Bool {
    let LlmModel(_, _, _, _, _, _, features, _, _, _, _, _) = model
    
    let has_code = not needs_code or list.any(features, fn(f) { f == CodeGeneration })
    let has_multimodal = not needs_multimodal or list.any(features, fn(f) { f == Multimodal })
    let has_realtime = not needs_realtime or list.any(features, fn(f) { f == RealTime })
    
    has_code and has_multimodal and has_realtime
  }
  
  let filtered = list.filter(all_models, fn(m) {
    filter_by_budget(m) and filter_by_features(m)
  })
  
  list.map(filtered, fn(m) {
    let LlmModel(provider, name, _, _, _, _, _, _, _, _, _, _) = m
    provider_to_string(provider) <> " - " <> name
  })
}
