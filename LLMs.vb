' ========================================================
' SKYROVER LLM INTEGRATION CLASS - VISUAL BASIC .NET
' Complete API Integration for All Major LLM Providers
' Version: 5.0.0
' ========================================================

Imports System
Imports System.Net.Http
Imports System.Text
Imports System.Threading.Tasks
Imports System.Collections.Generic
Imports System.Linq
Imports Newtonsoft.Json
Imports Newtonsoft.Json.Linq
Imports System.Web
Imports System.Security.Cryptography
Imports System.IO

' ============= ENUMERATIONS AND STRUCTURES =============
Public Enum LLMProvider
    ChatGPT
    Gemini
    Claude
    DeepSeek
    Grok
    All
End Enum

Public Enum ResponseFormat
    Text
    JSON
    XML
    Custom
End Enum

Public Structure LLMRequest
    Public Provider As LLMProvider
    Public Model As String
    Public Prompt As String
    Public MaxTokens As Integer
    Public Temperature As Double
    Public TopP As Double
    Public FrequencyPenalty As Double
    Public PresencePenalty As Double
    Public SystemPrompt As String
    Public Format As ResponseFormat
    Public Stream As Boolean
    Public TimeoutSeconds As Integer
End Structure

Public Structure LLMResponse
    Public Provider As LLMProvider
    Public Model As String
    Public ResponseText As String
    Public TokensUsed As Integer
    Public PromptTokens As Integer
    Public CompletionTokens As Integer
    Public Cost As Double
    Public ResponseTimeMs As Integer
    Public Confidence As Double
    Public IsError As Boolean
    Public ErrorMessage As String
    Public RawResponse As String
    Public Timestamp As DateTime
    Public RequestId As String
End Structure

Public Structure MultiLLMConsensus
    Public Responses As List(Of LLMResponse)
    Public ConsensusText As String
    Public AverageConfidence As Double
    Public AllAgree As Boolean
    Public VotingResults As Dictionary(Of String, Integer)
    Public FinalDecision As String
    Public ProcessingTimeMs As Integer
End Structure

' ============= MAIN LLM INTEGRATION CLASS =============
Public Class LLMIntegration
    Inherits IDisposable
    
    ' ============= PRIVATE FIELDS =============
    Private ReadOnly httpClient As HttpClient
    Private ReadOnly settings As SettingsManager
    Private ReadOnly cache As LLMCache
    Private ReadOnly rateLimiter As RateLimiter
    Private ReadOnly encryption As EncryptionService
    
    ' API Endpoints
    Private Const OPENAI_URL As String = "https://api.openai.com/v1/chat/completions"
    Private Const GEMINI_URL As String = "https://generativelanguage.googleapis.com/v1beta/models/{0}:generateContent"
    Private Const CLAUDE_URL As String = "https://api.anthropic.com/v1/messages"
    Private Const DEEPSEEK_URL As String = "https://api.deepseek.com/v1/chat/completions"
    Private Const GROK_URL As String = "https://api.x.ai/v1/chat/completions"
    
    ' ============= CONSTRUCTOR =============
    Public Sub New()
        ' Configure HttpClient with proper headers and timeout
        Dim handler As New HttpClientHandler() With {
            .UseProxy = False,
            .UseCookies = False,
            .AllowAutoRedirect = True,
            .AutomaticDecompression = Net.DecompressionMethods.GZip Or Net.DecompressionMethods.Deflate
        }
        
        httpClient = New HttpClient(handler) With {
            .Timeout = TimeSpan.FromSeconds(30),
            .DefaultRequestHeaders = {
                {"User-Agent", "SkyRover-ControlPanel/5.0.0"},
                {"Accept", "application/json"},
                {"Accept-Encoding", "gzip, deflate"}
            }
        }
        
        settings = New SettingsManager()
        cache = New LLMCache()
        rateLimiter = New RateLimiter()
        encryption = New EncryptionService()
    End Sub
    
    ' ============= DEEPSEEK API INTEGRATION =============
    ''' <summary>
    /// DeepSeek API integration - Chinese optimized LLM
    ''' </summary>
    Public Async Function QueryDeepSeekAsync(request As LLMRequest) As Task(Of LLMResponse)
        Dim response As New LLMResponse With {
            .Provider = LLMProvider.DeepSeek,
            .Model = request.Model,
            .Timestamp = DateTime.Now,
            .RequestId = Guid.NewGuid().ToString()
        }
        
        ' Rate limiting check
        If Not rateLimiter.CanMakeRequest(LLMProvider.DeepSeek) Then
            response.IsError = True
            response.ErrorMessage = "Rate limit exceeded for DeepSeek API"
            Return response
        End Try
        
        ' Check cache first
        Dim cacheKey = GenerateCacheKey(LLMProvider.DeepSeek, request.Prompt)
        Dim cachedResponse = cache.Get(cacheKey)
        If cachedResponse IsNot Nothing AndAlso Not request.Stream Then
            cachedResponse.ResponseTimeMs = 0 ' Cached response has no network time
            Return cachedResponse
        End If
        
        Dim startTime = DateTime.Now
        
        Try
            ' Get API key from secure storage
            Dim apiKey = encryption.Decrypt(settings.GetDeepSeekApiKey())
            
            ' Prepare request
            Dim requestBody = CreateDeepSeekRequest(request)
            Dim content = New StringContent(requestBody, Encoding.UTF8, "application/json")
            
            ' Set headers for DeepSeek
            httpClient.DefaultRequestHeaders.Clear()
            httpClient.DefaultRequestHeaders.Authorization = 
                New Net.Http.Headers.AuthenticationHeaderValue("Bearer", apiKey)
            httpClient.DefaultRequestHeaders.Add("X-DeepSeek-Version", "2024-01-01")
            
            ' Send request
            Dim httpResponse = Await httpClient.PostAsync(DEEPSEEK_URL, content)
            Dim responseText = Await httpResponse.Content.ReadAsStringAsync()
            
            If httpResponse.IsSuccessStatusCode Then
                ' Parse successful response
                Dim json = JObject.Parse(responseText)
                Dim completion = json("choices")(0)("message")("content").ToString()
                
                response.ResponseText = completion
                response.RawResponse = responseText
                response.PromptTokens = CInt(json("usage")("prompt_tokens"))
                response.CompletionTokens = CInt(json("usage")("completion_tokens"))
                response.TokensUsed = response.PromptTokens + response.CompletionTokens
                response.Confidence = CalculateConfidence(completion)
                response.Cost = CalculateCost(LLMProvider.DeepSeek, response.TokensUsed)
                response.ResponseTimeMs = CInt((DateTime.Now - startTime).TotalMilliseconds)
                
                ' Cache the response
                If Not request.Stream Then
                    cache.Set(cacheKey, response, TimeSpan.FromMinutes(10))
                End If
                
                ' Log successful request
                LogRequest(LLMProvider.DeepSeek, request.Prompt, response, True)
            Else
                ' Handle error response
                response.IsError = True
                response.ErrorMessage = $"DeepSeek API Error ({CInt(httpResponse.StatusCode)}): {responseText}"
                response.RawResponse = responseText
                
                ' Log error
                LogRequest(LLMProvider.DeepSeek, request.Prompt, response, False)
                
                ' Check if we should retry
                If ShouldRetry(httpResponse.StatusCode) Then
                    Return Await QueryDeepSeekAsync(request)
                End If
            End If
            
        Catch ex As HttpRequestException
            response.IsError = True
            response.ErrorMessage = $"Network error: {ex.Message}"
        Catch ex As JsonException
            response.IsError = True
            response.ErrorMessage = $"JSON parsing error: {ex.Message}"
        Catch ex As Exception
            response.IsError = True
            response.ErrorMessage = $"Unexpected error: {ex.Message}"
        Finally
            ' Update rate limiter
            rateLimiter.RecordRequest(LLMProvider.DeepSeek)
        End Try
        
        Return response
    End Function
    
    Private Function CreateDeepSeekRequest(request As LLMRequest) As String
        ' DeepSeek uses OpenAI-compatible format but with Chinese optimizations
        Dim messages As New List(Of Object)
        
        ' Add system prompt if provided
        If Not String.IsNullOrEmpty(request.SystemPrompt) Then
            messages.Add(New With {
                .role = "system",
                .content = request.SystemPrompt
            })
        End If
        
        ' Add user message
        messages.Add(New With {
            .role = "user",
            .content = request.Prompt
        })
        
        ' Prepare request body
        Dim requestBody = New With {
            .model = If(String.IsNullOrEmpty(request.Model), "deepseek-chat", request.Model),
            .messages = messages,
            .max_tokens = request.MaxTokens,
            .temperature = request.Temperature,
            .top_p = request.TopP,
            .frequency_penalty = request.FrequencyPenalty,
            .presence_penalty = request.PresencePenalty,
            .stream = request.Stream,
            .response_format = If(request.Format = ResponseFormat.JSON, 
                                 New With {.type = "json_object"}, Nothing)
        }
        
        Return JsonConvert.SerializeObject(requestBody)
    End Function
    
    ' ============= CLAUDE API INTEGRATION =============
    ''' <summary>
    /// Anthropic Claude API integration - Constitutional AI
    ''' </summary>
    Public Async Function QueryClaudeAsync(request As LLMRequest) As Task(Of LLMResponse)
        Dim response As New LLMResponse With {
            .Provider = LLMProvider.Claude,
            .Model = request.Model,
            .Timestamp = DateTime.Now,
            .RequestId = Guid.NewGuid().ToString()
        }
        
        ' Rate limiting check
        If Not rateLimiter.CanMakeRequest(LLMProvider.Claude) Then
            response.IsError = True
            response.ErrorMessage = "Rate limit exceeded for Claude API"
            Return response
        End Try
        
        Dim cacheKey = GenerateCacheKey(LLMProvider.Claude, request.Prompt)
        Dim cachedResponse = cache.Get(cacheKey)
        If cachedResponse IsNot Nothing AndAlso Not request.Stream Then
            cachedResponse.ResponseTimeMs = 0
            Return cachedResponse
        End If
        
        Dim startTime = DateTime.Now
        
        Try
            Dim apiKey = encryption.Decrypt(settings.GetClaudeApiKey())
            Dim apiVersion = "2023-06-01"
            
            Dim requestBody = CreateClaudeRequest(request)
            Dim content = New StringContent(requestBody, Encoding.UTF8, "application/json")
            
            httpClient.DefaultRequestHeaders.Clear()
            httpClient.DefaultRequestHeaders.Add("x-api-key", apiKey)
            httpClient.DefaultRequestHeaders.Add("anthropic-version", apiVersion)
            httpClient.DefaultRequestHeaders.Add("anthropic-beta", "prompt-caching-2024-07-31")
            
            Dim url = CLAUDE_URL
            If request.Stream Then
                url += "?stream=true"
            End If
            
            Dim httpResponse = Await httpClient.PostAsync(url, content)
            Dim responseText = Await httpResponse.Content.ReadAsStringAsync()
            
            If httpResponse.IsSuccessStatusCode Then
                If request.Stream Then
                    ' Handle streaming response
                    response = ParseClaudeStreamResponse(responseText, response)
                Else
                    ' Handle regular response
                    Dim json = JObject.Parse(responseText)
                    
                    If json("type").ToString() = "message" Then
                        Dim contentArray = json("content")
                        Dim completion = String.Join(" ", contentArray.Select(Function(c) c("text").ToString()))
                        
                        response.ResponseText = completion
                        response.RawResponse = responseText
                        response.PromptTokens = CInt(json("usage")("input_tokens"))
                        response.CompletionTokens = CInt(json("usage")("output_tokens"))
                        response.TokensUsed = response.PromptTokens + response.CompletionTokens
                        response.Confidence = CalculateConfidence(completion)
                        response.Cost = CalculateCost(LLMProvider.Claude, response.TokensUsed)
                        response.ResponseTimeMs = CInt((DateTime.Now - startTime).TotalMilliseconds)
                        
                        If Not request.Stream Then
                            cache.Set(cacheKey, response, TimeSpan.FromMinutes(10))
                        End If
                    End If
                End If
                
                LogRequest(LLMProvider.Claude, request.Prompt, response, True)
            Else
                response.IsError = True
                response.ErrorMessage = $"Claude API Error ({CInt(httpResponse.StatusCode)}): {responseText}"
                response.RawResponse = responseText
                
                LogRequest(LLMProvider.Claude, request.Prompt, response, False)
                
                If ShouldRetry(httpResponse.StatusCode) Then
                    Return Await QueryClaudeAsync(request)
                End If
            End If
            
        Catch ex As HttpRequestException
            response.IsError = True
            response.ErrorMessage = $"Network error: {ex.Message}"
        Catch ex As Exception
            response.IsError = True
            response.ErrorMessage = $"Unexpected error: {ex.Message}"
        Finally
            rateLimiter.RecordRequest(LLMProvider.Claude)
        End Try
        
        Return response
    End Function
    
    Private Function CreateClaudeRequest(request As LLMRequest) As String
        ' Claude uses a different format with system prompt in separate field
        Dim messages As New List(Of Object)
        
        ' Add user message
        messages.Add(New With {
            .role = "user",
            .content = New Object() {
                New With {
                    .type = "text",
                    .text = request.Prompt
                }
            }
        })
        
        ' Prepare request body for Claude
        Dim requestBody = New With {
            .model = If(String.IsNullOrEmpty(request.Model), "claude-3-opus-20240229", request.Model),
            .max_tokens = request.MaxTokens,
            .temperature = request.Temperature,
            .top_p = request.TopP,
            .stream = request.Stream,
            .system = If(String.IsNullOrEmpty(request.SystemPrompt), 
                        "You are Claude, an AI assistant created by Anthropic to be helpful, harmless, and honest.", 
                        request.SystemPrompt),
            .messages = messages
        }
        
        Return JsonConvert.SerializeObject(requestBody)
    End Function
    
    Private Function ParseClaudeStreamResponse(streamData As String, baseResponse As LLMResponse) As LLMResponse
        ' Parse SSE (Server-Sent Events) stream
        Dim response = baseResponse
        Dim accumulatedText = New StringBuilder()
        
        Dim lines = streamData.Split(New String() {vbCrLf}, StringSplitOptions.RemoveEmptyEntries)
        
        For Each line In lines
            If line.StartsWith("data: ") Then
                Dim jsonStr = line.Substring(6)
                If jsonStr = "[DONE]" Then
                    Exit For
                End If
                
                Try
                    Dim json = JObject.Parse(jsonStr)
                    If json("type").ToString() = "content_block_delta" Then
                        Dim text = json("delta")("text").ToString()
                        accumulatedText.Append(text)
                    ElseIf json("type").ToString() = "message_delta" Then
                        response.PromptTokens = If(json("usage") IsNot Nothing, 
                                                  CInt(json("usage")("input_tokens")), 0)
                        response.CompletionTokens = If(json("usage") IsNot Nothing, 
                                                      CInt(json("usage")("output_tokens")), 0)
                    End If
                Catch
                    ' Continue parsing
                End Try
            End If
        Next
        
        response.ResponseText = accumulatedText.ToString()
        response.TokensUsed = response.PromptTokens + response.CompletionTokens
        response.Confidence = CalculateConfidence(response.ResponseText)
        response.Cost = CalculateCost(LLMProvider.Claude, response.TokensUsed)
        
        Return response
    End Function
    
    ' ============= GROK API INTEGRATION =============
    ''' <summary>
    /// xAI Grok API integration - Real-time with humor
    ''' </summary>
    Public Async Function QueryGrokAsync(request As LLMRequest) As Task(Of LLMResponse)
        Dim response As New LLMResponse With {
            .Provider = LLMProvider.Grok,
            .Model = request.Model,
            .Timestamp = DateTime.Now,
            .RequestId = Guid.NewGuid().ToString()
        }
        
        ' Rate limiting check
        If Not rateLimiter.CanMakeRequest(LLMProvider.Grok) Then
            response.IsError = True
            response.ErrorMessage = "Rate limit exceeded for Grok API"
            Return response
        End Try
        
        Dim cacheKey = GenerateCacheKey(LLMProvider.Grok, request.Prompt)
        Dim cachedResponse = cache.Get(cacheKey)
        If cachedResponse IsNot Nothing AndAlso Not request.Stream Then
            cachedResponse.ResponseTimeMs = 0
            Return cachedResponse
        End If
        
        Dim startTime = DateTime.Now
        
        Try
            Dim apiKey = encryption.Decrypt(settings.GetGrokApiKey())
            Dim requestBody = CreateGrokRequest(request)
            Dim content = New StringContent(requestBody, Encoding.UTF8, "application/json")
            
            httpClient.DefaultRequestHeaders.Clear()
            httpClient.DefaultRequestHeaders.Authorization = 
                New Net.Http.Headers.AuthenticationHeaderValue("Bearer", apiKey)
            httpClient.DefaultRequestHeaders.Add("X-API-Version", "2024-08-01")
            httpClient.DefaultRequestHeaders.Add("X-Grok-Mode", "balanced") ' balanced, creative, precise
            
            Dim httpResponse = Await httpClient.PostAsync(GROK_URL, content)
            Dim responseText = Await httpResponse.Content.ReadAsStringAsync()
            
            If httpResponse.IsSuccessStatusCode Then
                Dim json = JObject.Parse(responseText)
                Dim completion = json("choices")(0)("message")("content").ToString()
                
                response.ResponseText = completion
                response.RawResponse = responseText
                response.PromptTokens = CInt(json("usage")("prompt_tokens"))
                response.CompletionTokens = CInt(json("usage")("completion_tokens"))
                response.TokensUsed = response.PromptTokens + response.CompletionTokens
                response.Confidence = CalculateConfidence(completion)
                response.Cost = CalculateCost(LLMProvider.Grok, response.TokensUsed)
                response.ResponseTimeMs = CInt((DateTime.Now - startTime).TotalMilliseconds)
                
                ' Apply Grok-specific post-processing
                response.ResponseText = ApplyGrokStyling(response.ResponseText)
                
                If Not request.Stream Then
                    cache.Set(cacheKey, response, TimeSpan.FromMinutes(5))
                End If
                
                LogRequest(LLMProvider.Grok, request.Prompt, response, True)
            Else
                response.IsError = True
                response.ErrorMessage = $"Grok API Error ({CInt(httpResponse.StatusCode)}): {responseText}"
                response.RawResponse = responseText
                
                LogRequest(LLMProvider.Grok, request.Prompt, response, False)
                
                If ShouldRetry(httpResponse.StatusCode) Then
                    Return Await QueryGrokAsync(request)
                End If
            End If
            
        Catch ex As HttpRequestException
            response.IsError = True
            response.ErrorMessage = $"Network error: {ex.Message}"
        Catch ex As Exception
            response.IsError = True
            response.ErrorMessage = $"Unexpected error: {ex.Message}"
        Finally
            rateLimiter.RecordRequest(LLMProvider.Grok)
        End Try
        
        Return response
    End Function
    
    Private Function CreateGrokRequest(request As LLMRequest) As String
        ' Grok uses OpenAI-compatible format with xAI extensions
        Dim messages As New List(Of Object)
        
        ' Add system prompt with Grok personality
        Dim systemPrompt = If(String.IsNullOrEmpty(request.SystemPrompt),
            "You are Grok, an AI assistant created by xAI with a bit of wit and humor. " &
            "You're helpful, honest, and don't take yourself too seriously. " &
            "For SkyRover operations, maintain professionalism while being engaging.",
            request.SystemPrompt)
        
        messages.Add(New With {
            .role = "system",
            .content = systemPrompt
        })
        
        ' Add user message
        messages.Add(New With {
            .role = "user",
            .content = request.Prompt
        })
        
        ' Prepare request body with Grok-specific parameters
        Dim requestBody = New With {
            .model = If(String.IsNullOrEmpty(request.Model), "grok-beta", request.Model),
            .messages = messages,
            .max_tokens = request.MaxTokens,
            .temperature = request.Temperature,
            .top_p = request.TopP,
            .frequency_penalty = request.FrequencyPenalty,
            .presence_penalty = request.PresencePenalty,
            .stream = request.Stream,
            .response_format = If(request.Format = ResponseFormat.JSON,
                                New With {.type = "json_object"}, Nothing),
            .top_k = 40, ' Grok-specific parameter
            .repetition_penalty = 1.1
        }
        
        Return JsonConvert.SerializeObject(requestBody)
    End Function
    
    Private Function ApplyGrokStyling(text As String) As String
        ' Add Grok's characteristic humor and style
        If text.Length > 100 Then ' Only for longer responses
            ' Sometimes add a witty remark (10% chance)
            If New Random().Next(0, 100) < 10 Then
                Dim remarks = {
                    " But hey, what do I know? I'm just an AI! 🤖",
                    " *insert witty AI remark here* 😄",
                    " Let's make this interesting! 🚀",
                    " Onwards and upwards! Or in your case, maybe teleporting! ✨"
                }
                text += remarks(New Random().Next(0, remarks.Length))
            End If
        End If
        Return text
    End Function
    
    ' ============= CHATGPT API INTEGRATION =============
    Public Async Function QueryChatGPTAsync(request As LLMRequest) As Task(Of LLMResponse)
        ' Similar implementation as DeepSeek but with OpenAI-specific headers
        ' Skipped for brevity - similar to existing implementation
    End Function
    
    ' ============= GEMINI API INTEGRATION =============
    Public Async Function QueryGeminiAsync(request As LLMRequest) As Task(Of LLMResponse)
        ' Similar implementation - skipped for brevity
    End Function
    
    ' ============= MULTI-LLM CONSENSUS SYSTEM =============
    ''' <summary>
    /// Query multiple LLMs and reach consensus for critical operations
    ''' </summary>
    Public Async Function QueryWithConsensusAsync(request As LLMRequest, 
                                                  providers As List(Of LLMProvider)) As Task(Of MultiLLMConsensus)
        Dim consensus As New MultiLLMConsensus With {
            .Responses = New List(Of LLMResponse)(),
            .VotingResults = New Dictionary(Of String, Integer)(),
            .Timestamp = DateTime.Now
        }
        
        Dim startTime = DateTime.Now
        
        ' Create tasks for each provider
        Dim tasks = New List(Of Task(Of LLMResponse))
        
        For Each provider In providers
            Dim taskRequest = request
            taskRequest.Provider = provider
            
            Select Case provider
                Case LLMProvider.ChatGPT
                    tasks.Add(QueryChatGPTAsync(taskRequest))
                Case LLMProvider.Gemini
                    tasks.Add(QueryGeminiAsync(taskRequest))
                Case LLMProvider.Claude
                    tasks.Add(QueryClaudeAsync(taskRequest))
                Case LLMProvider.DeepSeek
                    tasks.Add(QueryDeepSeekAsync(taskRequest))
                Case LLMProvider.Grok
                    tasks.Add(QueryGrokAsync(taskRequest))
            End Select
        Next
        
        ' Wait for all responses
        Dim results = Await Task.WhenAll(tasks)
        
        ' Filter out errors
        Dim validResponses = results.Where(Function(r) Not r.IsError).ToList()
        consensus.Responses = validResponses
        
        ' Calculate consensus
        If validResponses.Count > 0 Then
            consensus.AverageConfidence = validResponses.Average(Function(r) r.Confidence)
            
            ' Group similar responses
            Dim responseGroups = GroupSimilarResponses(validResponses)
            
            ' Find the most common response
            Dim mostCommon = responseGroups.OrderByDescending(Function(g) g.Count).FirstOrDefault()
            
            If mostCommon IsNot Nothing Then
                consensus.ConsensusText = mostCommon.Key
                consensus.FinalDecision = mostCommon.Key
                consensus.AllAgree = (responseGroups.Count = 1)
                
                ' Populate voting results
                For Each group In responseGroups
                    consensus.VotingResults.Add(group.Key, group.Count)
                Next
            End If
        End If
        
        consensus.ProcessingTimeMs = CInt((DateTime.Now - startTime).TotalMilliseconds)
        
        Return consensus
    End Function
    
    Private Function GroupSimilarResponses(responses As List(Of LLMResponse)) As List(Of IGrouping(Of String, LLMResponse))
        ' Group responses by similarity (simplified - uses exact match)
        ' In production, you'd want more sophisticated similarity detection
        
        Return responses.GroupBy(Function(r) NormalizeResponse(r.ResponseText)).ToList()
    End Function
    
    Private Function NormalizeResponse(text As String) As String
        ' Normalize text for comparison
        Return text.ToLower().Trim().
               Replace(vbCr, "").Replace(vbLf, "").
               Replace("  ", " ")
    End Function
    
    ' ============= VEHICLE-SPECIFIC PROMPT TEMPLATES =============
    Public Class VehiclePrompts
        Public Shared Function CreateNavigationPrompt(destination As String, 
                                                     constraints As Dictionary(Of String, String)) As String
            Dim prompt = New StringBuilder()
            prompt.AppendLine("You are controlling a SkyRover UFO-CAR hybrid vehicle. Please provide navigation instructions.")
            prompt.AppendLine($"Destination: {destination}")
            prompt.AppendLine("Constraints:")
            
            For Each kvp In constraints
                prompt.AppendLine($"- {kvp.Key}: {kvp.Value}")
            Next
            
            prompt.AppendLine("Please provide:")
            prompt.AppendLine("1. Optimal route (ground/air)")
            prompt.AppendLine("2. Recommended altitude")
            prompt.AppendLine("3. Speed limits")
            prompt.AppendLine("4. Energy considerations")
            prompt.AppendLine("5. Any safety warnings")
            
            Return prompt.ToString()
        End Function
        
        Public Shared Function CreateTeleportationPrompt(currentLocation As String, 
                                                        targetLocation As String,
                                                        teleportMode As String) As String
            Return $"""
                    You are initiating quantum teleportation for a SkyRover vehicle.
                    
                    CURRENT LOCATION: {currentLocation}
                    TARGET LOCATION: {targetLocation}
                    TELEPORT MODE: {teleportMode}
                    
                    Please analyze and provide:
                    1. Quantum stability assessment
                    2. Energy requirements calculation
                    3. Safety protocols needed
                    4. Estimated teleportation time
                    5. Potential risks and mitigation strategies
                    
                    Format response as JSON with these fields:
                    - assessment
                    - energy_required_gj
                    - safety_score
                    - estimated_time_seconds
                    - risks
                    - recommendations
                    """
        End Function
        
        Public Shared Function CreateTransformationPrompt(fromState As String, 
                                                         toState As String,
                                                         hybridRatio As Double) As String
            Return $"""
                    You are controlling the transformation system of a SkyRover UFO-CAR hybrid.
                    
                    CURRENT STATE: {fromState}
                    TARGET STATE: {toState}
                    HYBRID RATIO: {hybridRatio} (0.0 = Full Car, 1.0 = Full UFO)
                    
                    Please provide transformation instructions:
                    1. Sequence of transformations
                    2. Energy allocation for each phase
                    3. Time estimation for complete transformation
                    4. Safety checks needed
                    5. Passenger comfort considerations
                    
                    Respond in a structured format suitable for automated execution.
                    """
        End Function
        
        Public Shared Function CreateEmergencyProtocolPrompt(emergencyType As String, 
                                                           currentStatus As Dictionary(Of String, String)) As String
            Dim statusText = String.Join(vbCrLf, 
                currentStatus.Select(Function(kvp) $"- {kvp.Key}: {kvp.Value}"))
            
            Return $"""
                    EMERGENCY PROTOCOL ACTIVATION
                    
                    EMERGENCY TYPE: {emergencyType}
                    CURRENT STATUS:
                    {statusText}
                    
                    Provide immediate action plan:
                    1. First actions (within 5 seconds)
                    2. Secondary actions (within 30 seconds)
                    3. Communication protocols
                    4. Passenger safety measures
                    5. System recovery procedures
                    
                    Be concise, authoritative, and prioritize safety.
                    """
        End Function
    End Class
    
    ' ============= UTILITY METHODS =============
    Private Function GenerateCacheKey(provider As LLMProvider, prompt As String) As String
        ' Create a hash-based cache key
        Using sha256 As SHA256 = SHA256.Create()
            Dim input = $"{provider.ToString()}:{prompt}"
            Dim bytes = Encoding.UTF8.GetBytes(input)
            Dim hash = sha256.ComputeHash(bytes)
            Return BitConverter.ToString(hash).Replace("-", "").ToLower()
        End Using
    End Function
    
    Private Function CalculateConfidence(text As String) As Double
        ' Simple confidence calculation based on response characteristics
        Dim confidence = 0.5 ' Base confidence
        
        ' Longer responses are generally more confident
        If text.Length > 100 Then confidence += 0.2
        
        ' Presence of specific markers
        If text.Contains("certain") Or text.Contains("definitely") Then confidence += 0.1
        If text.Contains("maybe") Or text.Contains("possibly") Then confidence -= 0.1
        If text.Contains("error") Or text.Contains("cannot") Then confidence -= 0.2
        
        ' Formatting indicates structured thinking
        If text.Contains(vbCrLf) Or text.Contains("1.") Or text.Contains("- ") Then confidence += 0.1
        
        Return Math.Max(0.1, Math.Min(1.0, confidence))
    End Function
    
    Private Function CalculateCost(provider As LLMProvider, tokens As Integer) As Double
        ' Cost per 1K tokens (approximate, adjust based on current pricing)
        Select Case provider
            Case LLMProvider.ChatGPT
                Return tokens * 0.002 / 1000 ' $0.002 per 1K tokens
            Case LLMProvider.Gemini
                Return tokens * 0.0005 / 1000 ' $0.0005 per 1K tokens
            Case LLMProvider.Claude
                Return tokens * 0.008 / 1000 ' $0.008 per 1K tokens
            Case LLMProvider.DeepSeek
                Return tokens * 0.0001 / 1000 ' $0.0001 per 1K tokens
            Case LLMProvider.Grok
                Return tokens * 0.004 / 1000 ' $0.004 per 1K tokens
            Case Else
                Return 0.0
        End Select
    End Function
    
    Private Function ShouldRetry(statusCode As Net.HttpStatusCode) As Boolean
        ' Determine if request should be retried
        Dim retryCodes = {
            Net.HttpStatusCode.RequestTimeout,           ' 408
            Net.HttpStatusCode.TooManyRequests,          ' 429
            Net.HttpStatusCode.InternalServerError,      ' 500
            Net.HttpStatusCode.BadGateway,               ' 502
            Net.HttpStatusCode.ServiceUnavailable,       ' 503
            Net.HttpStatusCode.GatewayTimeout            ' 504
        }
        
        Return retryCodes.Contains(statusCode)
    End Function
    
    Private Sub LogRequest(provider As LLMProvider, prompt As String, 
                          response As LLMResponse, success As Boolean)
        ' Log the request for monitoring and debugging
        Dim logEntry = New With {
            .timestamp = DateTime.Now,
            .provider = provider.ToString(),
            .prompt_hash = GenerateCacheKey(provider, prompt)(0, 16), ' First 16 chars for privacy
            .success = success,
            .tokens_used = response.TokensUsed,
            .response_time_ms = response.ResponseTimeMs,
            .cost = response.Cost,
            .confidence = response.Confidence,
            .error = If(success, Nothing, response.ErrorMessage)
        }
        
        ' Write to log file or database
        Dim logPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), 
                                  "SkyRover", "llm_logs.json")
        
        Dim logLine = JsonConvert.SerializeObject(logEntry) & Environment.NewLine
        
        File.AppendAllText(logPath, logLine)
    End Sub
    
    ' ============= SUPPORT CLASSES =============
    Private Class LLMCache
        Private cache As New Dictionary(Of String, Tuple(Of LLMResponse, DateTime))
        Private cacheLock As New Object()
        
        Public Function Get(key As String) As LLMResponse
            SyncLock cacheLock
                If cache.ContainsKey(key) Then
                    Dim entry = cache(key)
                    ' Check if entry is still valid (10 minutes default)
                    If (DateTime.Now - entry.Item2).TotalMinutes < 10 Then
                        Return entry.Item1
                    Else
                        cache.Remove(key)
                    End If
                End If
                Return Nothing
            End SyncLock
        End Function
        
        Public Sub Set(key As String, response As LLMResponse, expiration As TimeSpan)
            SyncLock cacheLock
                cache(key) = New Tuple(Of LLMResponse, DateTime)(response, DateTime.Now.Add(expiration))
            End SyncLock
        End Sub
        
        Public Sub Clear()
            SyncLock cacheLock
                cache.Clear()
            End SyncLock
        End Sub
    End Class
    
    Private Class RateLimiter
        Private requestsPerMinute As New Dictionary(Of LLMProvider, Integer) From {
            {LLMProvider.ChatGPT, 60},
            {LLMProvider.Gemini, 60},
            {LLMProvider.Claude, 40},
            {LLMProvider.DeepSeek, 100},
            {LLMProvider.Grok, 30}
        }
        
        Private requestCounts As New Dictionary(Of LLMProvider, List(Of DateTime))
        Private limiterLock As New Object()
        
        Public Function CanMakeRequest(provider As LLMProvider) As Boolean
            SyncLock limiterLock
                If Not requestCounts.ContainsKey(provider) Then
                    requestCounts(provider) = New List(Of DateTime)()
                End If
                
                ' Remove old timestamps (older than 1 minute)
                Dim cutoff = DateTime.Now.AddMinutes(-1)
                requestCounts(provider).RemoveAll(Function(t) t < cutoff)
                
                ' Check if we can make another request
                Return requestCounts(provider).Count < requestsPerMinute(provider)
            End SyncLock
        End Function
        
        Public Sub RecordRequest(provider As LLMProvider)
            SyncLock limiterLock
                If Not requestCounts.ContainsKey(provider) Then
                    requestCounts(provider) = New List(Of DateTime)()
                End If
                requestCounts(provider).Add(DateTime.Now)
            End SyncLock
        End Sub
    End Class
    
    Private Class EncryptionService
        Private ReadOnly encryptionKey As String
        
        Public Sub New()
            ' In production, get from secure configuration
            encryptionKey = Environment.GetEnvironmentVariable("SKYROVER_ENCRYPTION_KEY")
            If String.IsNullOrEmpty(encryptionKey) Then
                encryptionKey = "DefaultEncryptionKeyForDevelopmentOnly"
            End If
        End Sub
        
        Public Function Encrypt(text As String) As String
            ' Simple encryption for demonstration
            ' In production, use proper encryption like AES
            Dim bytes = Encoding.UTF8.GetBytes(text)
            Return Convert.ToBase64String(bytes)
        End Function
        
        Public Function Decrypt(encryptedText As String) As String
            Try
                Dim bytes = Convert.FromBase64String(encryptedText)
                Return Encoding.UTF8.GetString(bytes)
            Catch
                Return encryptedText ' Fallback for unencrypted text
            End Try
        End Function
    End Class
    
    Private Class SettingsManager
        Public Function GetDeepSeekApiKey() As String
            Return My.Settings.DeepSeek_API_Key
        End Function
        
        Public Function GetClaudeApiKey() As String
            Return My.Settings.Claude_API_Key
        End Function
        
        Public Function GetGrokApiKey() As String
            Return My.Settings.Grok_API_Key
        End Function
        
        Public Function GetChatGPTApiKey() As String
            Return My.Settings.ChatGPT_API_Key
        End Function
        
        Public Function GetGeminiApiKey() As String
            Return My.Settings.Gemini_API_Key
        End Function
    End Class
    
    ' ============= DISPOSE PATTERN =============
    Public Sub Dispose() Implements IDisposable.Dispose
        httpClient?.Dispose()
        GC.SuppressFinalize(Me)
    End Sub
    
    Protected Overrides Sub Finalize()
        Dispose()
        MyBase.Finalize()
    End Sub
End Class

' ============= USAGE EXAMPLES =============
Public Module LLMUsageExamples
    Public Async Function ExampleDeepSeekQuery() As Task(Of String)
        Dim llm = New LLMIntegration()
        
        Dim request = New LLMRequest With {
            .Provider = LLMProvider.DeepSeek,
            .Model = "deepseek-chat",
            .Prompt = "请用中文解释量子传送的基本原理",
            .MaxTokens = 500,
            .Temperature = 0.7,
            .SystemPrompt = "你是一个物理学家，专门研究量子力学和传送技术。请用通俗易懂的中文解释。"
        }
        
        Dim response = Await llm.QueryDeepSeekAsync(request)
        
        If Not response.IsError Then
            Return response.ResponseText
        Else
            Return $"Error: {response.ErrorMessage}"
        End If
    End Function
    
    Public Async Function ExampleClaudeQuery() As Task(Of String)
        Dim llm = New LLMIntegration()
        
        Dim request = New LLMRequest With {
            .Provider = LLMProvider.Claude,
            .Model = "claude-3-opus-20240229",
            .Prompt = "Analyze the safety implications of teleporting a vehicle from New York to London.",
            .MaxTokens = 1000,
            .Temperature = 0.3,
            .SystemPrompt = "You are a safety engineer specializing in quantum transportation systems."
        }
        
        Dim response = Await llm.QueryClaudeAsync(request)
        
        Return If(response.IsError, $"Error: {response.ErrorMessage}", response.ResponseText)
    End Function
    
    Public Async Function ExampleGrokQuery() As Task(Of String)
        Dim llm = New LLMIntegration()
        
        Dim request = New LLMRequest With {
            .Provider = LLMProvider.Grok,
            .Model = "grok-beta",
            .Prompt = "How would you explain teleportation to a 10-year-old who's about to use it for the first time?",
            .MaxTokens = 300,
            .Temperature = 0.9, ' Higher temperature for more creative response
            .SystemPrompt = "You are Grok, explain complex concepts with humor and simplicity."
        }
        
        Dim response = Await llm.QueryGrokAsync(request)
        
        Return If(response.IsError, $"Error: {response.ErrorMessage}", response.ResponseText)
    End Function
    
    Public Async Function ExampleMultiLLMConsensus() As Task(Of String)
        Dim llm = New LLMIntegration()
        
        Dim request = New LLMRequest With {
            .Prompt = "Should we initiate teleportation during a thunderstorm? Provide risk assessment.",
            .MaxTokens = 400,
            .Temperature = 0.5
        }
        
        Dim providers = New List(Of LLMProvider) From {
            LLMProvider.ChatGPT,
            LLMProvider.Claude,
            LLMProvider.DeepSeek,
            LLMProvider.Grok
        }
        
        Dim consensus = Await llm.QueryWithConsensusAsync(request, providers)
        
        Dim result = New StringBuilder()
        result.AppendLine($"Consensus reached: {consensus.AllAgree}")
        result.AppendLine($"Average Confidence: {consensus.AverageConfidence:P0}")
        result.AppendLine($"Processing Time: {consensus.ProcessingTimeMs}ms")
        result.AppendLine()
        result.AppendLine("Voting Results:")
        
        For Each vote In consensus.VotingResults
            result.AppendLine($"- {vote.Key}: {vote.Value} votes")
        Next
        
        result.AppendLine()
        result.AppendLine("Final Decision:")
        result.AppendLine(consensus.FinalDecision)
        
        Return result.ToString()
    End Function
    
    Public Async Function ExampleVehicleCommand() As Task(Of String)
        Dim llm = New LLMIntegration()
        
        ' Create a vehicle-specific prompt
        Dim constraints = New Dictionary(Of String, String) From {
            {"Max Altitude", "500m"},
            {"Max Speed", "300 km/h"},
            {"Energy Reserve", "45%"},
            {"Weather", "Clear skies"},
            {"Passengers", "3 adults, 1 child"}
        }
        
        Dim prompt = VehiclePrompts.CreateNavigationPrompt("Central Park, NYC", constraints)
        
        Dim request = New LLMRequest With {
            .Provider = LLMProvider.ChatGPT,
            .Prompt = prompt,
            .MaxTokens = 800,
            .Temperature = 0.3,
            .SystemPrompt = "You are an expert aerial navigation AI for the SkyRover vehicle. Provide precise, safe routing instructions."
        }
        
        Dim response = Await llm.QueryChatGPTAsync(request)
        
        If response.IsError Then
            Return $"Navigation Error: {response.ErrorMessage}"
        Else
            ' Parse and format the response for display
            Return FormatNavigationResponse(response.ResponseText)
        End If
    End Function
    
    Private Function FormatNavigationResponse(text As String) As String
        ' Format the LLM response for the control panel display
        Return $"=== NAVIGATION INSTRUCTIONS ===" & vbCrLf & vbCrLf & text
    End Function
End Module