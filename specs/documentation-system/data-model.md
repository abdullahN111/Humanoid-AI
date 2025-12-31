# Data Model: Documentation System Structure

## Documentation Module
- **id**: Unique identifier for the module (string)
- **title**: Display title of the module (string)
- **description**: Brief description of the module content (string)
- **chapters**: Array of Chapter objects
- **order**: Numeric order for navigation (integer)
- **metadata**: Additional metadata for RAG indexing (object)

## Chapter
- **id**: Unique identifier for the chapter (string)
- **title**: Display title of the chapter (string)
- **content_path**: File path to the Markdown content (string)
- **module_id**: Reference to parent module (string)
- **order**: Numeric order within the module (integer)
- **tags**: Array of tags for categorization (array of strings)
- **metadata**: Additional metadata for RAG indexing (object)

## Navigation Item
- **type**: Type of navigation item (string: 'category', 'doc', 'link')
- **label**: Display label for the navigation item (string)
- **items**: Array of child navigation items (array of Navigation Item objects)
- **docId**: Reference to a specific document (string, optional)
- **href**: External link (string, optional)

## Content Metadata
- **title**: Document title (string)
- **description**: Document description (string)
- **tags**: Array of tags for the document (array of strings)
- **authors**: Array of author names (array of strings)
- **created_date**: Date when content was created (date)
- **updated_date**: Date when content was last updated (date)
- **version**: Content version (string)
- **status**: Publication status (string: 'draft', 'published', 'archived')

## RAG Indexing Data
- **content_id**: Unique identifier for the content chunk (string)
- **module_id**: Reference to the module (string)
- **chapter_id**: Reference to the chapter (string)
- **content**: The actual text content for indexing (string)
- **headings**: Array of headings in the content (array of strings)
- **embedding**: Vector embedding of the content (array of numbers)
- **metadata**: Additional metadata for search (object)

## File Structure Mapping
The data model maps to the following file structure:

```
docs/
├── module-1/                    # Module directory
│   ├── _category_.json         # Module metadata
│   ├── chapter-1.md            # Chapter content
│   ├── chapter-2.md            # Chapter content
│   └── chapter-3.md            # Chapter content
├── module-2/
│   ├── _category_.json
│   ├── chapter-1.md
│   └── ...
└── ...
```

## Relationships
- Module 1 -----> * Chapter (one-to-many)
- Chapter 1 -----> 1 Module (many-to-one)
- Module 1 -----> 1 Navigation Item (one-to-one for primary navigation)
- Chapter 1 -----> * Navigation Item (one-to-many if chapters appear in multiple places)
- Chapter 1 -----> * RAG Indexing Data (one-to-many for content chunks)