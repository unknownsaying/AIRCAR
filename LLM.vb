Public Class LLMIntegration
    Private ReadOnly httpClient As New HttpClient()
    
    Public Async Function QueryChatGPTAsync(prompt As String) As Task(Of String)
        Try
            Dim apiKey = My.Settings.OpenAI_API_Key
            httpClient.DefaultRequestHeaders.Authorization = 
                New Net.Http.Headers.AuthenticationHeaderValue("Bearer", apiKey)
            
            Dim requestBody As String = JsonConvert.SerializeObject(New With {
                .model = "gpt-4",
                .messages = {New With {.role = "user", .content = prompt}},
                .temperature = 0.7,
                .max_tokens = 500
            })
            
            Dim content As New StringContent(requestBody, Encoding.UTF8, "application/json")
            Dim response = Await httpClient.PostAsync("https://api.openai.com/v1/chat/completions", content)
            
            If response.IsSuccessStatusCode Then
                Dim json = Await response.Content.ReadAsStringAsync()
                Dim result = JsonConvert.DeserializeObject(Of JObject)(json)
                Return result("choices")(0)("message")("content").ToString()
            End If
            
        Catch ex As Exception
            Return $"ChatGPT Error: {ex.Message}"
        End Try
        
        Return "No response from ChatGPT"
    End Function
    
    Public Async Function QueryGeminiAsync(prompt As String) As Task(Of String)
        Try
            Dim apiKey = My.Settings.Google_AI_API_Key
            Dim requestBody As String = JsonConvert.SerializeObject(New With {
                .contents = {New With {
                    .parts = {New With {.text = prompt}}
                }}
            })
            
            Dim content As New StringContent(requestBody, Encoding.UTF8, "application/json")
            Dim url = $"https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key={apiKey}"
            Dim response = Await httpClient.PostAsync(url, content)
            
            If response.IsSuccessStatusCode Then
                Dim json = Await response.Content.ReadAsStringAsync()
                Dim result = JsonConvert.DeserializeObject(Of JObject)(json)
                Return result("candidates")(0)("content")("parts")(0)("text").ToString()
            End If
            
        Catch ex As Exception
            Return $"Gemini Error: {ex.Message}"
        End Try
        
        Return "No response from Gemini"
    End Function
    
    Public Async Function QueryAllLLMsAsync(prompt As String) As Task(Of List(Of LLMResponse))
        Dim tasks As New List(Of Task(Of LLMResponse))
        
        tasks.Add(Task.Run(Async Function()
                               Dim response = Await QueryChatGPTAsync(prompt)
                               Return New LLMResponse With {
                                   .Provider = LLMProvider.ChatGPT,
                                   .Command = prompt,
                                   .Response = response,
                                   .Confidence = 0.95
                               }
                           End Function))
        
        tasks.Add(Task.Run(Async Function()
                               Dim response = Await QueryGeminiAsync(prompt)
                               Return New LLMResponse With {
                                   .Provider = LLMProvider.Gemini,
                                   .Command = prompt,
                                   .Response = response,
                                   .Confidence = 0.92
                               }
                           End Function))
        
        ' Add Claude, DeepSeek, Grok similarly
        
        Dim results = Await Task.WhenAll(tasks)
        Return results.ToList()
    End Function
End Class