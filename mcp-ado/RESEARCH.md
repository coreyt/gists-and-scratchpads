# MCP Server for Azure DevOps - Research & Approaches

This document summarizes approaches to create a local MCP (Model Context Protocol) server that interfaces with Azure DevOps (git repos, work items, pull requests, pipelines, etc.).

## Executive Summary

There are **three main approaches** to get an MCP server for Azure DevOps:

| Approach | Effort | Recommended For |
|----------|--------|-----------------|
| 1. Use Microsoft's Official MCP Server | Minimal | Production use, full feature set |
| 2. Use Community MCP Server | Minimal | Alternative auth options, customization base |
| 3. Build Custom MCP Server | High | Specialized needs, learning MCP |

**Recommendation**: Use Microsoft's official Azure DevOps MCP Server unless you have specific requirements it doesn't meet.

---

## Approach 1: Microsoft's Official Azure DevOps MCP Server (Recommended)

### Overview
Microsoft released an official Azure DevOps MCP Server (GA as of October 2025) that runs locally and provides secure access to Azure DevOps resources.

### Key Features
- **Runs locally** - Sensitive project data never leaves your network
- **Browser-based authentication** - Uses Microsoft account login
- **Comprehensive coverage** - Projects, work items, PRs, builds, test plans, wiki, pipelines
- **Domain filtering** - Load only the tool groups you need
- **Free to use** - Only standard Azure DevOps pricing applies

### Installation

```bash
# Via npx (no installation needed)
npx -y @azure-devops/mcp <organization-name>

# Or install globally
npm install -g @azure-devops/mcp
```

### Configuration for VS Code

Create `.vscode/mcp.json`:

```json
{
  "inputs": [{
    "id": "ado_org",
    "type": "promptString",
    "description": "Azure DevOps organization name (e.g. 'contoso')"
  }],
  "servers": {
    "ado": {
      "type": "stdio",
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "${input:ado_org}"]
    }
  }
}
```

### Configuration for Claude Desktop

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "your-org-name"]
    }
  }
}
```

### Available Domains (Tool Groups)

Use `-d` flag to load specific domains:

```json
"args": ["-y", "@azure-devops/mcp", "org-name", "-d", "core", "work-items", "repositories"]
```

| Domain | Description |
|--------|-------------|
| `core` | Project-level information (recommended baseline) |
| `work` | Work management (sprints, iterations, teams) |
| `work-items` | Work item CRUD operations |
| `search` | Code and work item search |
| `test-plans` | Test plan management |
| `repositories` | Git repository operations |
| `wiki` | Wiki page management |
| `pipelines` | Build and release pipelines |
| `advanced-security` | Security scanning results |

### Example Capabilities
- "List my ADO projects"
- "List ADO Repos for 'Contoso'"
- "List my work items for project 'Contoso'"
- "Create a wiki page '/Architecture/Overview'"
- "List iterations for project 'Contoso'"

### Links
- GitHub: https://github.com/microsoft/azure-devops-mcp
- Docs: https://learn.microsoft.com/en-us/azure/devops/mcp-server/mcp-server-overview
- Blog: https://devblogs.microsoft.com/devops/azure-devops-mcp-server-public-preview/

### Limitations
- Only supports Azure DevOps Services (cloud), not on-premises Azure DevOps Server

---

## Approach 2: Community MCP Server (Tiberriver256)

### Overview
A community-built alternative that provides more authentication options and can serve as a customization base.

### Key Features
- **Multiple auth methods** - PAT, Azure Identity (DefaultAzureCredential), Azure CLI
- **Service principal support** - Good for automation scenarios
- **Modular architecture** - Easier to extend/customize
- **MIT licensed** - Full source available

### Installation

```bash
npx @tiberriver256/mcp-server-azure-devops
```

### Configuration

Environment variables:

```bash
# Required
AZURE_DEVOPS_ORG_URL=https://dev.azure.com/your-org

# Authentication (choose one method)
AZURE_DEVOPS_AUTH_METHOD=pat  # or 'azure-identity' or 'azure-cli'

# For PAT authentication
AZURE_DEVOPS_PAT=your-personal-access-token

# For Service Principal (azure-identity)
AZURE_CLIENT_ID=your-client-id
AZURE_TENANT_ID=your-tenant-id
AZURE_CLIENT_SECRET=your-client-secret
```

### Claude Desktop Configuration

```json
{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["@tiberriver256/mcp-server-azure-devops"],
      "env": {
        "AZURE_DEVOPS_ORG_URL": "https://dev.azure.com/your-org",
        "AZURE_DEVOPS_AUTH_METHOD": "pat",
        "AZURE_DEVOPS_PAT": "your-pat-token"
      }
    }
  }
}
```

### Available Capabilities
- **Projects**: List, get details
- **Repositories**: List, get files, browse trees, create branches, commits
- **Work Items**: Create, update, search, link
- **Pull Requests**: Create, comment, update status, view changes
- **Pipelines**: Run, get artifacts, view logs, timelines
- **Wiki**: Get pages, search content
- **Search**: Code search, work item search

### Links
- GitHub: https://github.com/Tiberriver256/mcp-server-azure-devops
- NPM: https://www.npmjs.com/package/@tiberriver256/mcp-server-azure-devops

---

## Approach 3: Build Custom MCP Server

### When to Build Custom
- Need specialized tools not in existing servers
- Want to integrate with additional systems alongside ADO
- Learning MCP protocol/architecture
- Need on-premises Azure DevOps Server support

### Technology Choices

#### TypeScript (Recommended)
- Strong typing for tool definitions
- Fast iteration with Node.js ecosystem
- Official SDK well-maintained

```bash
npm install @modelcontextprotocol/server zod
```

#### Python
- FastMCP provides high-level abstractions
- Good for data science/ML integration

```bash
pip install mcp
```

### MCP Core Concepts

1. **Tools** - Functions the LLM can invoke (side effects, API calls)
2. **Resources** - Read-only data exposed to clients
3. **Prompts** - Reusable templates for consistent interactions
4. **Transports** - Communication layer (stdio, HTTP)

### Basic TypeScript Server Structure

```typescript
import { McpServer } from "@modelcontextprotocol/server";
import { StdioServerTransport } from "@modelcontextprotocol/server/transports";
import { z } from "zod";

const server = new McpServer({
  name: "azure-devops-custom",
  version: "1.0.0",
});

// Define a tool
server.tool(
  "list_projects",
  "List all projects in the Azure DevOps organization",
  {
    // Input schema (using Zod)
  },
  async (params) => {
    // Call Azure DevOps REST API
    // Return results
  }
);

// Connect transport
const transport = new StdioServerTransport();
await server.connect(transport);
```

### Azure DevOps REST API Reference

Base URL: `https://dev.azure.com/{organization}`

| Resource | Endpoint | Method |
|----------|----------|--------|
| Projects | `/_apis/projects` | GET |
| Repositories | `/{project}/_apis/git/repositories` | GET |
| Work Items | `/_apis/wit/workitems/{id}` | GET |
| Work Item Query | `/{project}/_apis/wit/wiql` | POST |
| Pull Requests | `/{project}/_apis/git/repositories/{repo}/pullrequests` | GET/POST |
| Pipelines | `/{project}/_apis/pipelines` | GET |
| Builds | `/{project}/_apis/build/builds` | GET |

API Version: `api-version=7.1`

### Authentication Options

1. **Personal Access Token (PAT)**
   ```typescript
   const headers = {
     'Authorization': `Basic ${Buffer.from(`:${pat}`).toString('base64')}`
   };
   ```

2. **Azure Identity (DefaultAzureCredential)**
   ```typescript
   import { DefaultAzureCredential } from "@azure/identity";
   const credential = new DefaultAzureCredential();
   const token = await credential.getToken("499b84ac-1321-427f-aa17-267ca6975798/.default");
   ```

3. **Azure CLI**
   ```bash
   az login
   az account get-access-token --resource 499b84ac-1321-427f-aa17-267ca6975798
   ```

### Project Structure Example

```
mcp-ado-custom/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts           # Entry point, server setup
│   ├── auth/
│   │   └── credentials.ts # Authentication handling
│   ├── tools/
│   │   ├── projects.ts    # Project-related tools
│   │   ├── repos.ts       # Repository tools
│   │   ├── workitems.ts   # Work item tools
│   │   └── pullrequests.ts# PR tools
│   └── types/
│       └── ado.ts         # TypeScript types for ADO responses
```

### SDK Resources
- TypeScript SDK: https://github.com/modelcontextprotocol/typescript-sdk
- Python SDK: https://github.com/modelcontextprotocol/python-sdk
- MCP Specification: https://spec.modelcontextprotocol.io
- Example Servers: https://github.com/modelcontextprotocol/servers

---

## Comparison Matrix

| Feature | Microsoft Official | Community (Tiberriver256) | Custom Build |
|---------|-------------------|---------------------------|--------------|
| Setup Time | Minutes | Minutes | Days/Weeks |
| Maintenance | Microsoft | Community | You |
| Auth Options | Browser/MS Account | PAT, Azure Identity, CLI | Flexible |
| Customization | Limited | Fork & modify | Full control |
| On-Prem Support | No | Possible | Yes |
| Documentation | Excellent | Good | You write it |
| Production Ready | Yes | Yes | Depends |

---

## Quick Start Recommendation

For most users wanting to integrate Azure DevOps with Claude or other AI assistants:

```bash
# 1. Install (nothing to install, npx handles it)
# 2. Configure Claude Desktop (~/.config/claude/claude_desktop_config.json on Linux)

{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "your-organization-name", "-d", "core", "work-items", "repositories"]
    }
  }
}

# 3. Restart Claude Desktop
# 4. First tool use will prompt for Microsoft login
```

---

## Use Case Analysis: Activity Queries

This section analyzes specific query patterns for tracking user and project activity.

### Query 1: "What work did corey@email.com perform over the past month?"

This query requires aggregating data across multiple Azure DevOps domains:

| Data Type | API Support | Server-Side Filtering | Notes |
|-----------|-------------|----------------------|-------|
| **Work Items** | ✅ Excellent | ✅ Yes | WIQL supports `[Changed By]`, `[Created By]`, `[Assigned To]` + date ranges |
| **Commits** | ✅ Good | ✅ Yes | `searchCriteria.author` + `fromDate`/`toDate` parameters |
| **PR Comments** | ⚠️ Partial | ❌ No | Must fetch all threads, filter client-side by `author.uniqueName` |
| **PR Created/Reviewed** | ✅ Good | ✅ Yes | `searchCriteria.creatorId` or `reviewerId` parameters |
| **Pipeline Runs** | ⚠️ Partial | ❌ No | No direct user filter; must check `requestedFor` client-side |

#### Work Items - Full Server-Side Support

```
POST /_apis/wit/wiql
{
  "query": "SELECT [System.Id], [System.Title], [System.State]
            FROM WorkItems
            WHERE [System.ChangedBy] = 'corey@email.com'
            AND [System.ChangedDate] >= @Today - 30"
}
```

**Caveat**: WIQL returns only work item IDs. A second call to `/_apis/wit/workitems?ids=1,2,3` is needed for full details.

#### Commits - Full Server-Side Support

```
GET /{project}/_apis/git/repositories/{repo}/commits
    ?searchCriteria.author=corey@email.com
    &searchCriteria.fromDate=2025-12-29
    &searchCriteria.toDate=2026-01-29
```

**Caveat**: Must iterate over all repositories in the org/project.

#### PR Comments - Client-Side Filtering Required

```
# Step 1: Get all PRs (can filter by date range)
GET /_apis/git/repositories/{repo}/pullrequests?searchCriteria.status=all

# Step 2: For each PR, get threads
GET /_apis/git/repositories/{repo}/pullRequests/{prId}/threads

# Step 3: Filter comments where author.uniqueName == 'corey@email.com'
```

**Problem**: This is **N+1 queries** - one per PR. For active repos, this is slow and rate-limit-prone.

---

### Query 2: "What work was done on project Z this week?"

| Data Type | API Support | Server-Side Filtering | Notes |
|-----------|-------------|----------------------|-------|
| **Work Items** | ✅ Excellent | ✅ Yes | WIQL scoped to project + date range |
| **Commits** | ✅ Excellent | ✅ Yes | All repos in project, date range filter |
| **Pull Requests** | ✅ Excellent | ✅ Yes | Date range + status filters |
| **Pipeline Runs** | ✅ Good | ✅ Yes | `minTime`/`maxTime` parameters |

This query is **much better supported** because project-scoping is native to most endpoints.

#### Work Items

```
POST /{project}/_apis/wit/wiql
{
  "query": "SELECT [System.Id] FROM WorkItems
            WHERE [System.TeamProject] = 'ProjectZ'
            AND [System.ChangedDate] >= @Today - 7"
}
```

#### All Commits Across Repos

```
# Get all repos in project
GET /{project}/_apis/git/repositories

# For each repo, get commits in date range
GET /{project}/_apis/git/repositories/{repo}/commits
    ?searchCriteria.fromDate=2026-01-22
```

---

### API Limitations & Performance Concerns

#### Rate Limits
- **Global limit**: 200 TSTUs (throughput units) per 5-minute window
- **Symptoms**: `Retry-After` header in response, HTTP 429
- **Mitigation**: Honor `Retry-After`, implement exponential backoff

#### Pagination
- Most endpoints return max 100-200 items per request
- Use continuation tokens or `$skip`/`$top` parameters
- Large result sets require multiple round-trips

#### N+1 Query Problem (PR Comments)
The biggest issue for user-centric queries is **PR comment retrieval**:

```
PRs in org          × Threads per PR    × API calls
    500             ×      10           = 5,000+ calls
```

**Solutions**:
1. **Cache aggressively** - Comments don't change after creation
2. **Limit scope** - Only query recent/active PRs
3. **Use Search API** - `/_apis/search/codesearchresults` may help for some use cases
4. **Accept latency** - Run as batch job, not real-time

#### WIQL Limitations
- Max 32KB query length
- `WAS EVER` / `EVER` operators scan all revisions (slow)
- `ASOF` queries for historical snapshots add overhead
- Returns only IDs - second call needed for field data

---

### Can an MCP Server Help?

**Yes, but with caveats.**

#### What MCP Provides
1. **Unified interface** - Single protocol for AI to query ADO
2. **Tool abstraction** - LLM doesn't need to know API details
3. **Authentication handling** - MCP server manages tokens

#### What MCP Doesn't Solve
1. **API limitations** - Still bound by ADO REST API capabilities
2. **N+1 queries** - MCP server still makes the same underlying calls
3. **Rate limits** - MCP adds no protection against throttling

#### Recommended Architecture for Activity Queries

```
┌─────────────────────────────────────────────────────────────┐
│                     User Query                              │
│  "What did corey@email.com do last month?"                  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    MCP Server                               │
│  - Exposes high-level tools like:                           │
│    • get_user_activity(email, date_range)                   │
│    • get_project_summary(project, date_range)               │
└─────────────────────────────────────────────────────────────┘
                              │
            ┌─────────────────┼─────────────────┐
            ▼                 ▼                 ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│  Work Items   │   │    Commits    │   │  PR Comments  │
│   (WIQL)      │   │  (per-repo)   │   │ (cached/batch)│
└───────────────┘   └───────────────┘   └───────────────┘
            │                 │                 │
            └─────────────────┼─────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Local Cache / DB                          │
│  - SQLite or similar for PR comments, commit history        │
│  - Incremental sync to avoid repeated full fetches          │
└─────────────────────────────────────────────────────────────┘
```

#### Practical Recommendations

| Approach | Effort | Best For |
|----------|--------|----------|
| **Use existing MCP + LLM reasoning** | Low | Ad-hoc queries, small projects |
| **Custom MCP with aggregation tools** | Medium | Regular activity reports |
| **MCP + background sync + local cache** | High | Large orgs, real-time dashboards |

For the specific queries mentioned:
1. **Start with Microsoft's MCP server** - Test if existing tools suffice
2. **If too slow** - Build custom tools that pre-aggregate data
3. **For enterprise scale** - Consider Azure DevOps Analytics Service (OData) as data source instead of REST API

---

### Alternative: Azure DevOps Analytics Service (OData)

For complex activity queries, consider the **Analytics Service** instead of REST API:

```
GET https://analytics.dev.azure.com/{org}/{project}/_odata/v4.0-preview/WorkItemRevisions
    ?$filter=ChangedDate ge 2026-01-01 and ChangedBy/UserEmail eq 'corey@email.com'
    &$select=WorkItemId,Title,State,ChangedDate
```

**Advantages**:
- Designed for reporting/analytics workloads
- Better aggregation support
- Less rate limiting for read-heavy queries

**Disadvantages**:
- Read-only (no mutations)
- Data may be slightly delayed (not real-time)
- Different API structure to learn

---

## Sources

### MCP Servers
- [Microsoft Azure DevOps MCP Server - GitHub](https://github.com/microsoft/azure-devops-mcp)
- [Azure DevOps MCP Server Overview - Microsoft Learn](https://learn.microsoft.com/en-us/azure/devops/mcp-server/mcp-server-overview)
- [Community MCP Server - GitHub](https://github.com/Tiberriver256/mcp-server-azure-devops)
- [MCP TypeScript SDK - GitHub](https://github.com/modelcontextprotocol/typescript-sdk)
- [Build MCP Servers with TypeScript - DEV Community](https://dev.to/shadid12/how-to-build-mcp-servers-with-typescript-sdk-1c28)

### Azure DevOps REST API
- [WIQL Syntax Reference - Microsoft Learn](https://learn.microsoft.com/en-us/azure/devops/boards/queries/wiql-syntax?view=azure-devops)
- [WIQL Query API - Microsoft Learn](https://learn.microsoft.com/en-us/rest/api/azure/devops/wit/wiql/query-by-wiql?view=azure-devops-rest-7.1)
- [Get Commits API - Microsoft Learn](https://learn.microsoft.com/en-us/rest/api/azure/devops/git/commits/get-commits?view=azure-devops-rest-7.1)
- [Pull Request Threads API - Microsoft Learn](https://learn.microsoft.com/en-us/rest/api/azure/devops/git/pull-request-threads?view=azure-devops-rest-7.1)
- [Rate and Usage Limits - Microsoft Learn](https://learn.microsoft.com/en-us/azure/devops/integrate/concepts/rate-limits?view=azure-devops)
- [Integration Best Practices - Microsoft Learn](https://learn.microsoft.com/en-us/azure/devops/integrate/concepts/integration-bestpractices?view=azure-devops)
