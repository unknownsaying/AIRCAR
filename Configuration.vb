Imports System.Configuration
Imports System.IO

Public Class AppSettingsManager
    Private Shared ReadOnly settings As Configuration
    
    Shared Sub New()
        Try
            Dim configFile = "SkyRover.config"
            Dim map = New ExeConfigurationFileMap() With {
                .ExeConfigFilename = configFile
            }
            settings = ConfigurationManager.OpenMappedExeConfiguration(map, ConfigurationUserLevel.None)
        Catch
            ' Use default configuration
            settings = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None)
        End Try
    End Sub
    
    Public Shared Function GetSetting(key As String, Optional defaultValue As String = "") As String
        Try
            Dim setting = settings.AppSettings.Settings(key)
            If setting IsNot Nothing Then
                Return setting.Value
            End If
        Catch ex As Exception
            ' Log error
        End Try
        
        Return defaultValue
    End Function
    
    Public Shared Sub SaveSetting(key As String, value As String)
        Try
            If settings.AppSettings.Settings(key) Is Nothing Then
                settings.AppSettings.Settings.Add(key, value)
            Else
                settings.AppSettings.Settings(key).Value = value
            End If
            
            settings.Save(ConfigurationSaveMode.Modified)
            ConfigurationManager.RefreshSection("appSettings")
        Catch ex As Exception
            ' Handle error
        End Try
    End Sub
    
    ' API Key management with encryption
    Public Shared Function GetApiKey(provider As LLMProvider) As String
        Dim keyName = GetProviderKeyName(provider)
        Dim encryptedKey = GetSetting(keyName)
        
        If String.IsNullOrEmpty(encryptedKey) Then
            Return String.Empty
        End If
        
        ' Decrypt the API key
        Try
            Return EncryptionHelper.Decrypt(encryptedKey, GetMachineKey())
        Catch
            Return encryptedKey ' Return as-is if decryption fails
        End Try
    End Function
    
    Public Shared Sub SaveApiKey(provider As LLMProvider, apiKey As String)
        Dim keyName = GetProviderKeyName(provider)
        
        ' Encrypt before saving
        Dim encryptedKey = EncryptionHelper.Encrypt(apiKey, GetMachineKey())
        SaveSetting(keyName, encryptedKey)
    End Sub
    
    Private Shared Function GetProviderKeyName(provider As LLMProvider) As String
        Return $"{provider.ToString()}_API_Key"
    End Function
    
    Private Shared Function GetMachineKey() As String
        ' Generate a machine-specific key for encryption
        Dim machineInfo = $"{Environment.MachineName}{Environment.UserName}"
        Using sha256 = Security.Cryptography.SHA256.Create()
            Dim hash = sha256.ComputeHash(Encoding.UTF8.GetBytes(machineInfo))
            Return Convert.ToBase64String(hash)
        End Using
    End Function
End Class